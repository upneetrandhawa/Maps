#include <unordered_map>
#include "StreetsDatabaseAPI.h"
#include "m1.h"
#include <vector>
#include <algorithm>
#include <math.h>
#include <set>
#include <unordered_map>

//vector of vector getting street segment id from intersection id
std::vector<std::vector<unsigned>> intersection_id_street_segment_id;

//map which contains street name id,  value vector of street ids
std::unordered_map<std::string, std::vector<unsigned>> map_street_name_street_ids;

//map which contains key-street id, value vector containing street segment ids
std::unordered_map<unsigned,std::vector<unsigned> > street_id_street_segment_ids;

std::vector<StreetSegmentInfo> street_id_info;

//map which contains key-street name, value vector containing intersection  ids
std::unordered_map<std::string, std::vector<unsigned> > map_street_name_intersection_id;
//load the map

std::vector<LatLon> intersection_id_LatLon;

bool load_map(std::string map_name) {

    bool load_success = loadStreetsDatabaseBIN(map_name);


    // create any data structures here to speed up your API functions
    // ...
/******************************************************************************************************************************************/

    // Iterating through all intersection ids
    for (unsigned intersectionId = 0; intersectionId < getNumberOfIntersections(); intersectionId++) {
        
        //getting the total number of street segment 
        unsigned number_of_street_segments_at_intersection = getIntersectionStreetSegmentCount(intersectionId);
        
        // vector of street segments to be stored in intersection_id_street_segment_id 
        std::vector<unsigned> street_segments;
        for (unsigned j = 0; j < number_of_street_segments_at_intersection; j++) {
            unsigned streetSegmentId = getIntersectionStreetSegment(intersectionId, j);
            street_segments.push_back(streetSegmentId);
        }
        
        // store street_segments for each intersection_id
        intersection_id_street_segment_id.push_back(street_segments);
        
        //storing intersection positions(LatLon) in intersection_id_LatLon
        intersection_id_LatLon.push_back(getIntersectionPosition(intersectionId));
    
    }
    
    /************************************************************************************************************************************/
    //creating map SSG_id_street_id
    for (unsigned i = 0; i < getNumberOfStreetSegments(); i++) {

        street_id_info.push_back( getStreetSegmentInfo(i));
        street_id_street_segment_ids[getStreetSegmentInfo(i).streetID].push_back(i);
        
    }

/******************************************************************************************************************************************/
 
    for (unsigned streetId = 0; streetId < getNumberOfStreets(); streetId++) {
      
      // creating map map_street_name_intersection_id
        std::string street_name = getStreetName(streetId);
        std::vector<unsigned>  intersection_vector_at_street_id= find_all_street_intersections(streetId);
        std::vector<unsigned>  intersection_value = map_street_name_intersection_id[street_name];
        intersection_value.insert(intersection_value.end(), intersection_vector_at_street_id.begin(),
                                    intersection_vector_at_street_id.end());
        map_street_name_intersection_id[street_name] = intersection_value;
    }
/******************************************************************************************************************************************/
    
    for (unsigned streetId = 0; streetId < getNumberOfStreets(); streetId++) {
      
        map_street_name_street_ids[getStreetName(streetId)].push_back(streetId);
       
    }

    return load_success;


}

std::vector<unsigned> find_street_ids_from_name(std::string street_name) {
    
    return map_street_name_street_ids[street_name];
}
//close the map

void close_map() {
    closeStreetDatabase();

    // destroy/clear any data structures you created in load_map
    // ...
}

std::vector<unsigned> find_intersection_street_segments(unsigned intersection_id) {
    
    return intersection_id_street_segment_id[intersection_id];
}

bool are_directly_connected(unsigned intersection_id1, unsigned intersection_id2) { //Upneet 
    if (intersection_id1 == intersection_id2)
        return true;

    std::vector<unsigned> intersection_id1_segments = find_intersection_street_segments(intersection_id1); // vector of street segments at intersection id 1
    std::vector<unsigned> intersection_id2_segments = find_intersection_street_segments(intersection_id2); // vector of street segments at intersection id 2

    std::vector<unsigned> result;

    std::sort(intersection_id1_segments.begin(), intersection_id1_segments.end());
    std::sort(intersection_id2_segments.begin(), intersection_id2_segments.end());
    std::set_intersection(intersection_id1_segments.begin(), intersection_id1_segments.end(), intersection_id2_segments.begin(), intersection_id2_segments.end(), std::back_inserter(result));

    if (result.size() != 0)
        return true;
    else
        return false;
}



//Implemented by Pankaj: Unit test: Functionality -> Pass : Timint -> Pass
//find distance between two coordinates

double find_distance_between_two_points(LatLon point1, LatLon point2) {

    //Structure of LatLon structure for information copied from LatLon.h file.
    /*** Latitude and longitude in decimal degrees
    float lat = std::numeric_limits<float>::quiet_NaN();
    float lon = std::numeric_limits<float>::quiet_NaN();
     */

    //converting to radians
    double average_latitude_radians = DEG_TO_RAD * ((point1.lat + point2.lat) / 2);

    double x_coordinate_point_1 = ((DEG_TO_RAD * point1.lon) * cos(average_latitude_radians));
    double y_coordinate_point_1 = DEG_TO_RAD * point1.lat;

    double x_coordinate_point_2 = ((DEG_TO_RAD * point2.lon) * cos(average_latitude_radians));
    double y_coordinate_point_2 = DEG_TO_RAD * point2.lat;

    //Calculated as Radius of earth * Squareroot((x2-x1)^2 + (y2-y1^2)
    double distance_between_two_points = EARTH_RADIUS_IN_METERS *
            sqrt(((x_coordinate_point_2 - x_coordinate_point_1)*(x_coordinate_point_2 - x_coordinate_point_1))
            +((y_coordinate_point_2 - y_coordinate_point_1)*(y_coordinate_point_2 - y_coordinate_point_1)));

    return distance_between_two_points;
}

std::vector<std::string> find_intersection_street_names(unsigned intersection_id) {
    std::vector<unsigned> intersection_id_segments = find_intersection_street_segments(intersection_id); // vector of street segments at intersection id

    std::vector<std::string> result;
    for (unsigned i = 0; i < intersection_id_segments.size(); i++) {
        result.push_back(getStreetName(getStreetSegmentInfo(intersection_id_segments[i]).streetID));

    }
    return result;

}

std::vector<unsigned> find_street_street_segments(unsigned street_id) {

    return street_id_street_segment_ids[street_id];

}

std::vector<unsigned> find_adjacent_intersections(unsigned intersection_id) {

        std::vector<unsigned> street_segments_ids = find_intersection_street_segments(intersection_id); // vector of street segments at intersection ids
        std::vector<unsigned> vec;

        for (unsigned j = 0; j < street_segments_ids.size(); j++) {
            StreetSegmentInfo SSinfo = getStreetSegmentInfo(street_segments_ids[j]);
            unsigned from, to;
            from = SSinfo.from;
            to = SSinfo.to;

            if (are_directly_connected(to, intersection_id))
                vec.push_back(to);

            if ( SSinfo.oneWay == false)
                vec.push_back(from);
        }
        std::sort(vec.begin(), vec.end());
        vec.erase(unique(vec.begin(), vec.end()), vec.end());

        std::vector<unsigned>::iterator position = std::find(vec.begin(), vec.end(), intersection_id);
        if (position != vec.end()) // == vec.end() means the element was not found
            vec.erase(position);

    return vec;
}




//Implemented by Pankaj: Unit test: Functionality -> Pass : Timint -> Pass

double find_street_segment_length(unsigned street_segment_id) {

    double street_segment_length = 0;

    //    OSMID wayOSMID; // OSM ID of the source way
    //    unsigned from, to; // intersection ID this segment runs from/to
    //    bool oneWay; // if true, then can only travel in from->to direction
    //    unsigned curvePointCount; // number of curve points between the ends
    //    float speedLimit;
    //    unsigned streetID; // index of street this segment belongs to

    StreetSegmentInfo street_Segment_Info = getStreetSegmentInfo(street_segment_id);

    LatLon intersection_1 = getIntersectionPosition(street_Segment_Info.from);
    LatLon intersection_2 = getIntersectionPosition(street_Segment_Info.to);
    std::vector<LatLon> points_to_pass;
    points_to_pass.push_back(intersection_1);

    for (unsigned curve_point_number = 0; curve_point_number < (street_Segment_Info.curvePointCount); curve_point_number++) {
        points_to_pass.push_back(getStreetSegmentCurvePoint(street_segment_id, curve_point_number));
    }
    points_to_pass.push_back(intersection_2);


    //Now vector points_to_pass data type LatLon contains all the points to pass
    //Calculating all distances and adding them together.
    for (std::vector<LatLon>::iterator it = points_to_pass.begin(); it != (points_to_pass.end() - 1); ++it) {
        street_segment_length += find_distance_between_two_points(*it, *(it + 1));
    }

    return street_segment_length;
}

std::vector<unsigned> find_all_street_intersections(unsigned street_id) {
        std::vector<unsigned> vec = find_street_street_segments(street_id);
        std::vector<unsigned> value;
        std::vector<unsigned> fromVec, toVec;

        for (unsigned j = 0; j < vec.size(); j++) {
            StreetSegmentInfo SSinfo = street_id_info[vec[j]];
            unsigned from = SSinfo.from;
            unsigned to = SSinfo.to;


            value.push_back(from);
            value.push_back(to);
        }

        std::sort(value.begin(), value.end());
        value.erase(unique(value.begin(), value.end()), value.end());
  
       return value;
}


//find the length of a whole street

double find_street_length(unsigned street_id) {
    double street_length = 0;
    std::vector<unsigned> street_segments_ids = find_street_street_segments(street_id);

    for (std::vector<unsigned>::iterator it = street_segments_ids.begin(); it != (street_segments_ids.end()); ++it) {
        street_length += find_street_segment_length(*it);
    }
    return street_length;
}


//find the travel time to drive a street segment (time(minutes) = distance(km)/speed_limit(km/hr) * 60

double find_street_segment_travel_time(unsigned street_segment_id) {
    double street_segment_travel_time = find_street_segment_length(street_segment_id) / (getStreetSegmentInfo(street_segment_id).speedLimit * 1000)*60;

    return street_segment_travel_time;
}

std::vector<unsigned> find_intersection_ids_from_street_names(std::string street_name1, std::string street_name2) {

    std::vector<unsigned> intersection_id1 = map_street_name_intersection_id[street_name1];
    std::vector<unsigned> intersection_id2 = map_street_name_intersection_id[street_name2];

    std::vector<unsigned> result;

    std::sort(intersection_id1.begin(), intersection_id1.end());
    std::sort(intersection_id2.begin(), intersection_id2.end());
    std::set_intersection(intersection_id1.begin(), intersection_id1.end(), intersection_id2.begin(), intersection_id2.end(), std::back_inserter(result));

    return result;  

}

unsigned find_closest_point_of_interest(LatLon my_position) {
    std::vector <double> distance;
    double minimumDistance = find_distance_between_two_points(my_position, getPointOfInterestPosition(0));
    unsigned closestPOI = 0;
    for (unsigned i = 1; i < getNumberOfPointsOfInterest(); i++) {

        LatLon POI_position = getPointOfInterestPosition(i);
        //        distance.push_back(find_distance_between_two_points(my_position,POI_position));
        double distance = find_distance_between_two_points(my_position, POI_position);
        if (distance < minimumDistance) {
            minimumDistance = distance;
            closestPOI = i;
        }
    }
    return closestPOI;

}

unsigned find_closest_intersection(LatLon my_position) {
 
    double minimumDistance = find_distance_between_two_points(my_position, intersection_id_LatLon[0]);
    unsigned closest_intersection = 0;
    unsigned i =0;
    for (; i< intersection_id_LatLon.size(); i++) {

        LatLon intersection_position = intersection_id_LatLon[i];
        //        distance.push_back(find_distance_between_two_points(my_position,POI_position));
        double distance = find_distance_between_two_points(my_position, intersection_position);
        if (distance < minimumDistance) {
            minimumDistance = distance;
            closest_intersection = i;
        }
    }

    return closest_intersection;
}


//std::vector <unsigned> find_closest_longitude(float my_longitude, LatLon my_position) {
//
//    int lowerBound = 0, upperBound = intersection_id_longitude.size();
//    std::vector <unsigned> closest_intersection_ids;
//    int mid;
//
//    while (lowerBound <= upperBound) {
//        mid = (lowerBound + upperBound) / 2;
//
//        if (my_longitude == intersection_id_longitude[mid].first) {
//            closest_intersection_ids.push_back(intersection_id_longitude[mid].second);
//            upperBound = mid;
//        } else if (my_longitude < intersection_id_longitude[mid].first)
//            upperBound = mid;
//
//        else if (my_longitude > intersection_id_longitude[mid].first)
//            lowerBound = mid + 1;
//
//        if (upperBound == (lowerBound + 1)) {
//
//
//            double dist_lb = find_distance_between_two_points(my_position, intersection_id_LatLon[intersection_id_longitude[lowerBound].second]);
//            double dist_ub = find_distance_between_two_points(my_position, intersection_id_LatLon[intersection_id_longitude[upperBound].second]);
//
//            if (dist_lb < dist_ub) {
//
//                closest_intersection_ids.push_back(intersection_id_longitude[lowerBound].second);
//
//            } else if (dist_lb > dist_ub) {
//                closest_intersection_ids.push_back(intersection_id_longitude[upperBound].second);
//            } else {
//                closest_intersection_ids.push_back(intersection_id_longitude[lowerBound].second);
//                closest_intersection_ids.push_back(intersection_id_longitude[upperBound].second);
//            }
//        }
//
//
//    }
//    return closest_intersection_ids;
//
//}
//
//std::vector <unsigned> find_closest_latitude(float my_latitude, LatLon my_position) {
//
//    int lowerBound = 0, upperBound = intersection_id_latitude.size();
//    std::vector <unsigned> closest_intersection_ids;
//    int mid;
//
//    while (lowerBound <= upperBound) {
//        mid = (lowerBound + upperBound) / 2;
//
//        if (my_latitude == intersection_id_latitude[mid].first) {
//            closest_intersection_ids.push_back(intersection_id_latitude[mid].second);
//            upperBound = mid;
//        } else if (my_latitude < intersection_id_latitude[mid].first)
//            upperBound = mid;
//
//        else if (my_latitude > intersection_id_latitude[mid].first)
//            lowerBound = mid + 1;
//
//        if (upperBound == (lowerBound + 1)) {
//
//
//            double dist_lb = find_distance_between_two_points(my_position, intersection_id_LatLon[intersection_id_latitude[lowerBound].second]);
//            double dist_ub = find_distance_between_two_points(my_position, intersection_id_LatLon[intersection_id_latitude[upperBound].second]);
//
//            if (dist_lb < dist_ub) {
//
//                closest_intersection_ids.push_back(intersection_id_latitude[lowerBound].second);
//
//            } else if (dist_lb > dist_ub) {
//                closest_intersection_ids.push_back(intersection_id_latitude[upperBound].second);
//            } else {
//                closest_intersection_ids.push_back(intersection_id_latitude[lowerBound].second);
//                closest_intersection_ids.push_back(intersection_id_latitude[upperBound].second);
//            }
//        }
//
//
//    }
//    return closest_intersection_ids;
//
//}
//
//unsigned find_closest_intersection(LatLon my_position) {
//    float my_latitude = my_position.lat;
//    float my_longitude = my_position.lon;
//
//    std::vector <unsigned> closest_intersection_ids_latitude = find_closest_latitude(my_latitude, my_position);
//    std::vector <unsigned> closest_intersection_ids_longitude = find_closest_latitude(my_longitude, my_position);
//
//    double minimumDistance1 = find_distance_between_two_points(my_position, intersection_id_LatLon[closest_intersection_ids_latitude[0]]);
//    
//    unsigned closest_intersection1 = 0;
//    
//    for (unsigned i = 1 ; i<  closest_intersection_ids_latitude.size() ; i++){
//         LatLon intersection_position = intersection_id_LatLon[i];
//        //        distance.push_back(find_distance_between_two_points(my_position,POI_position));
//        double distance = find_distance_between_two_points(my_position, intersection_position);
//        if (distance < minimumDistance1) {
//            minimumDistance1 = distance;
//            closest_intersection1 = i;
//        }
//    }
//    
//    double minimumDistance2 = find_distance_between_two_points(my_position, intersection_id_LatLon[closest_intersection_ids_longitude[0]]);
//    
//    unsigned closest_intersection2 = 0;
//    
//    for (unsigned i = 1 ; i<  closest_intersection_ids_longitude.size() ; i++){
//         LatLon intersection_position = intersection_id_LatLon[i];
//        //        distance.push_back(find_distance_between_two_points(my_position,POI_position));
//        double distance = find_distance_between_two_points(my_position, intersection_position);
//        if (distance < minimumDistance1) {
//            minimumDistance2 = distance;
//            closest_intersection2 = i;
//        }
//    }
//    if (minimumDistance1 == minimumDistance2)
//        return closest_intersection2;
//    else if (minimumDistance1 < minimumDistance2)
//        return closest_intersection1;
//    else
//        return closest_intersection2;    
//}   

