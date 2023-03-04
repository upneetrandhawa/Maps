#include "StreetsDatabaseAPI.h"
#include "m1.h"

using namespace std;

int main() {

    load_map("/cad2/ece297s/public/maps/toronto.streets.bin");
    
   
    // try out your m1.h functions here
    
    cout<<" "<<find_closest_intersection(LatLon(43.51741, -79.78996)) << endl;
    
    close_map();
    
    return 0;
}