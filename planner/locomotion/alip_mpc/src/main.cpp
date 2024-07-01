#include <iostream>
#include <unistd.h> // usleep
#include <time.h>


#include "data_structs.hpp"

#include "NewStep_mpc.hpp"


using namespace std;

int main(int argc, char *argv[])
{

    // Create standard input/output structs
    input_data_t input_data;
    output_data_t output_data;
    full_horizon_sol fdata;
    // Initialize foot placement controller
    string step_horizon = "4";
    string intervals = "4";
    NewStep_mpc newStep_mpc(step_horizon, intervals);
    cout << "--> ALIP-MPC Foot Placement Controller Initialized!\n";

    // Timing Variables
    clock_t start;
    double comp_time;

    /********************* MAIN LOOP *****************/
    

    
    while (true)
    {   

        //RECEIVE INFORMATION FROM SIMULATINO OR ROBOT TO INPUT DATA

        // Foot placement calculation
        cout << "horizon: " + step_horizon << "\n";
        newStep_mpc.Update_(input_data, output_data, fdata);
        cout << "Update complete\n";


    } 
    return 0;
}
