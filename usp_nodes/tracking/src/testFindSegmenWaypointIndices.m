current_position = [1,2]
flight_plan = [0,0; 1,1; 3,2; 4,6; 9,7]

[a_index,b_index, distance_to_line, distance_to_point_a, distance_to_point_b] = findSegmentWaypointsIndices(current_position, flight_plan);

disp(a_index)
disp(b_index)
disp(distance_to_line)
disp(distance_to_point_a)
disp(distance_to_point_b)

function [a_index, b_index, distance_to_line, distance_to_point_a, distance_to_point_b] = findSegmentWaypointsIndices(current_position, flight_plan)

    distance_to_line = Inf;
    tam = size(flight_plan);
    tam = tam(1) - 1;
    for index = 1:tam
       waypoint_a = flight_plan(index,:);
       waypoint_b = flight_plan(index+1,:);
       
       vector_1 = current_position - waypoint_a;
       vector_2 = waypoint_b - waypoint_a;
       
       angle = acos((vector_1*vector_2')/(norm(vector_1)*norm(vector_2)));
       
       if (angle < pi/2 && angle > -pi/2)
           distance = distanceFromPointToLine(current_position, waypoint_a, vector_2);
           if distance < distance_to_line
               distance_to_line = distance;
               first_index = index;
               second_index = index+1;
           end
       end
    end
    
    a_index = first_index;
    b_index = second_index;
    
    waypoint_a = flight_plan(a_index,:);
    waypoint_b = flight_plan(b_index,:);
    
    distance_to_point_a = distanceBetweenPoints(waypoint_a, current_position);
    distance_to_point_b = distanceBetweenPoints(waypoint_b, current_position);
end

function distance = distanceFromPointToLine(current_position, waypoint_a, vector_2)
    distance = norm(cross( [(waypoint_a - current_position) 0], [vector_2 0] ))/norm(vector_2);
end

function distance = distanceBetweenPoints(point_a, point_b)
    distance = sqrt((point_a(1) - point_b(1))^2 + (point_a(2) - point_b(2))^2);
end
