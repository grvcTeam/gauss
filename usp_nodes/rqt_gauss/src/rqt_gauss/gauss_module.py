import os
import copy
import rospkg
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtWidgets import QWidget, QMessageBox
from gauss_msgs_mqtt.msg import UTMAlternativeFlightPlan, RPSFlightPlanAccept, RPSChangeFlightStatus
from gauss_msgs.srv import ReadIcao, ReadIcaoRequest
from gauss_light_sim.srv import ChangeFlightPlan, ChangeFlightPlanRequest

class GaussPlugin(Plugin):
    pop_up = Signal()

    def __init__(self, context):
        super(GaussPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('GaussPlugin')
        rp = rospkg.RosPack()
        rospy.loginfo('[RQt] Started Gauss RQt node!')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print '[RQt] arguments: ', args
            print '[RQt] unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(rp.get_path('rqt_gauss'), 'resource', 'GaussPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('GaussPluginUi')

        # Do connections and stuff here. For complex plugins, consider
        # creating custom helper classes instead of QWidget
        self.status_pub = rospy.Publisher('/gauss/flight', RPSChangeFlightStatus, queue_size=1)
        self.acceptance_pub = rospy.Publisher('/gauss/flightacceptance', RPSFlightPlanAccept, queue_size=10)
        rospy.Subscriber('/gauss/alternative_flight_plan', UTMAlternativeFlightPlan, self.alternative_flight_plan_callback,queue_size = 10)
        self.read_icao_service = rospy.ServiceProxy('/gauss/read_icao', ReadIcao)

        self._widget.refresh_button.clicked.connect(self.handle_refresh_button_clicked)
        self._widget.start_button.clicked.connect(self.handle_start_button_clicked)
        self._widget.stop_button.clicked.connect(self.handle_stop_button_clicked)
        self.pop_up.connect(self.show_pop_up)
        self.alternative_flight_plan = None

        self.handle_refresh_button_clicked()  # Try to refresh here

        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def handle_refresh_button_clicked(self):
        try:
            read_icao_response = self.read_icao_service.call(ReadIcaoRequest())

            self._widget.icao_list.clear()
            for icao in read_icao_response.icao_address:
                self._widget.icao_list.addItem(icao)

        except rospy.ServiceException as e:
            print("[RQt] Service call failed: %s"%e)

    def change_flight_status(self, status):
        selected_icao = self._widget.icao_list.currentItem()
        if selected_icao is None:
            rospy.logerr('[RQt] Select an icao address')
            return
        icao = selected_icao.text()

        change_flight_msg = RPSChangeFlightStatus()
        change_flight_msg.icao = int(icao)
        # change_flight_msg.flight_plan_id  # TODO
        change_flight_msg.status = status
        # change_flight_msg.timestamp  # TODO

        rospy.loginfo('[RQt] RPA[{}]: mission {}'.format(icao, status))
        self.status_pub.publish(change_flight_msg)

    def handle_start_button_clicked(self):
        self.change_flight_status('start')

    def handle_stop_button_clicked(self):
        self.change_flight_status('stop')

    def alternative_flight_plan_callback(self, data):
        # if self.alternative_flight_plan is None:
            self.alternative_flight_plan = data
            rospy.loginfo('[RQt] Received: [{}]'.format(data))
            self.pop_up.emit()
        # else:
        #     rospy.loginfo('[RQt] Waiting for pilot response...')

    def show_pop_up(self):
        msg = QMessageBox()
        msg.setWindowTitle('Alternative flight plan received')
        msg.setText('Accept alternative flight plan for [' + str(self.alternative_flight_plan.icao) + '] ?')
        msg.setIcon(QMessageBox.Question)
        msg.setStandardButtons(QMessageBox.Yes|QMessageBox.No)
        msg.setDefaultButton(QMessageBox.No)
        # msg.setInformativeText('Accept new plan?')
        msg.setDetailedText('{}'.format(self.alternative_flight_plan))
        # msg.buttonClicked.connect(self.handle_message_box)  # TODO: do handling here?
        pop_up_response = msg.exec_()
        ros_response = RPSFlightPlanAccept()
        ros_response.icao = self.alternative_flight_plan.icao
        ros_response.flight_plan_id = self.alternative_flight_plan.flight_plan_id

        if pop_up_response == QMessageBox.Yes:
            ros_response.accept = True
            # Try to call light_sim service to change flight plan
            try:
                light_sim_change_flight_plan = rospy.ServiceProxy('/gauss_light_sim/change_flight_plan', ChangeFlightPlan)
                light_sim_change_flight_plan(ChangeFlightPlanRequest(alternative=copy.deepcopy(self.alternative_flight_plan)))
            except rospy.ServiceException as e:
                print("[RQt] Service call failed: %s"%e)

        if pop_up_response == QMessageBox.No:
            ros_response.accept = False
        self.acceptance_pub.publish(ros_response)
        self.alternative_flight_plan = None

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
