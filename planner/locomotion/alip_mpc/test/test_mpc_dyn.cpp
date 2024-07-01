#include <iostream>
#include <time.h>
#include <vector>
#include <bits/stdc++.h> 
#include <stdlib.h>

#include "test_mpc_dyn.hpp"
#include "../include/NewStep_mpc.hpp"
#include "../include/data_structs.hpp"

using namespace std;

void func_to_test(test_data_t ** tdata, input_data_t indata, output_data_t outdata, int i){
    //x_lip_current
    tdata[i]->x_c = indata.xlip_current[0];
    tdata[i]->y_c = indata.xlip_current[1];
    tdata[i]->L_xc = indata.xlip_current[2];
    tdata[i]->L_yc = indata.xlip_current[3];

    tdata[i]->stance_leg = indata.stance_leg;
    tdata[i]->zH = indata.zH;
    tdata[i]->Ts = indata.Ts;
    tdata[i]->Tr = indata.Tr;
    tdata[i]->leg_width = indata.leg_width;
    tdata[i]->Lx_offset = indata.Lx_offset;
    tdata[i]->Ly_des = indata.Ly_des;
    tdata[i]->kx = indata.kx;
    tdata[i]->ky = indata.ky;
    tdata[i]->mu = indata.mu;


    tdata[i]->ufp_wrt_st_x = outdata.ufp_wrt_st[0];
    tdata[i]->ufp_wrt_st_y = outdata.ufp_wrt_st[1];

    tdata[i]->ufp_wrt_com_x = outdata.ufp_wrt_st[0];
    tdata[i]->ufp_wrt_com_y = outdata.ufp_wrt_st[1];

}

void initialize_data(input_data_t &indata, output_data_t &outdata, double Lx, double Ly){
    indata.xlip_current[0]= -0.1;
    indata.xlip_current[1]= 0;
    indata.xlip_current[2]= 0;
    indata.xlip_current[3]= 0;

    indata.stance_leg = -1;
    indata.zH = 0.7;
    indata.Ts = 0.25;
    indata.Tr = 0.25;
    indata.leg_width= 0.13;
    indata.Lx_offset = Lx;
    indata.Ly_des = Ly;
    indata.kx = 0;
    indata.ky = 0;
    indata.mu = 0.3;

    outdata.ufp_wrt_st[0] = 0;
    outdata.ufp_wrt_st[1] = 0;

    outdata.ufp_wrt_com[0] = 0;
    outdata.ufp_wrt_st[1] = 0;
}

void printTestData(struct test_data_t* data) {
    //printf("time: %f\n", data->time);
    //printf("s: %f\n", data->s);
    printf("x_c: %f\n", data->x_c);
    printf("y_c: %f\n", data->y_c);
    printf("L_xc: %f\n", data->L_xc);
    printf("L_yc: %f\n", data->L_yc);
    printf("stance_leg: %f\n", data->stance_leg);
    printf("zH: %f\n", data->zH);
    printf("Ts: %f\n", data->Ts);
    printf("Tr: %f\n", data->Tr);
    printf("leg_width: %f\n", data->leg_width);
    printf("Lx_offset: %f\n", data->Lx_offset);
    printf("Ly_des: %f\n", data->Ly_des);
    printf("kx: %f\n", data->kx);
    printf("ky: %f\n", data->ky);
    printf("mu: %f\n", data->mu);
    printf("ufp_wrt_st_x: %f\n", data->ufp_wrt_st_x);
    printf("ufp_wrt_st_y: %f\n", data->ufp_wrt_st_y);
    printf("ufp_wrt_com_x: %f\n", data->ufp_wrt_com_x);
    printf("ufp_wrt_com_y: %f\n", data->ufp_wrt_com_y);
    printf("---\n");
}

void newinputData(input_data_t &inputdata, full_horizon_sol data){
    for(int i = 0; i < 4; i++){
        inputdata.xlip_current[i] = data.xlip_sol[i];
    }
    inputdata.stance_leg *= -1;
}



int main(){
    FILE* pd;
    FILE* pdd;

    //choose number of iterations
    int test_it = 1;

    test_data_t *tdata[test_it+1];
    // Initialization of tdata pointers
    for (int i = 0; i <= test_it; ++i) {
        tdata[i] = new test_data_t;      // Allocate memory for each element
    }

    input_data_t indata;
    output_data_t outdata;
    full_horizon_sol fullmpcdata;

    //choose step horizon and intevals
    string step_horizon = "4";
    string intervals = "4";
    //choose duration of test, number of iterations
    NewStep_mpc newStep_mpc(step_horizon, intervals, false);

    initialize_data(indata, outdata, 2 , 5);


    func_to_test(tdata, indata, outdata, 0);

    for (int i = 0; i < test_it; i++){
        newStep_mpc.Update_(indata, outdata, fullmpcdata);
        func_to_test(tdata, indata, outdata, 1);
        newinputData(indata, fullmpcdata);
    }


    cout << "results" << endl;
    cout << "indata" << endl;
    printTestData(tdata[0]);
    cout << "resulta" << endl << endl;
    printTestData(tdata[1]);

    // Open and write the binary to plot in python for input/output data
    pd = fopen("Update_test.bin", "wb");
    if (pd == NULL) {
        printf("Update_test cannot be created\n");
    }
    for (int i = 0; i <= test_it; ++i) {
        fwrite(tdata[i], sizeof(struct test_data_t), 1, pd);
    }
    fclose(pd);

    // Free allocated memory for tdata[i] structures
    for (int i = 0; i <= test_it; ++i) {
        free(tdata[i]);
    }
    printf("Data saved to Update_test.bin\n");

    //works for test it = 1, if not the data will e for the last mpc iteration
    ofstream file("Full_Mpc_sol.bin", ios::binary);
    if (file.is_open()) {
        // Save the size of vectors
        size_t xlip_size = fullmpcdata.xlip_sol.size();
        size_t ufp_size = fullmpcdata.ufp_sol.size();
        file.write(reinterpret_cast<char*>(&xlip_size), sizeof(size_t)); //reinterpret_cast is used to convert the size_t variables and vector data to char* for binary file writing
        file.write(reinterpret_cast<char*>(&ufp_size), sizeof(size_t));

        // Save vector data
        file.write(reinterpret_cast<char*>(fullmpcdata.xlip_sol.data()), xlip_size * sizeof(double));  
        file.write(reinterpret_cast<char*>(fullmpcdata.ufp_sol.data()), ufp_size * sizeof(double));

        file.close();
        cout << "Data saved to Full_Mpc_sol.bin\n";
    } else {
        cerr << "Failed to open the file.\n";
    }
   

    //print some data
    //cout << fullmpcdata.xlip_sol.size() << " size " << "hey" << endl;
    //cout << "ufp sol:" << endl;
    //for (int i = 0; i < stoi(step_horizon); i++) cout << fullmpcdata.ufp_sol[2*i] << " " << fullmpcdata.ufp_sol[2*i+1] << endl;
    //cout << endl;
    //cout << "x + ufp" << endl;
    /*
    for (int i = 0; i < stoi(step_horizon); i++){
        cout << fullmpcdata.xlip_sol[4*i]+fullmpcdata.ufp_sol[2*i];
        cout << "  " << fullmpcdata.xlip_sol[4*i+1]+fullmpcdata.ufp_sol[2*i+1] << endl;
    }
    cout << " L: " << endl;
    for (int i = 0; i < stoi(step_horizon) ;i++){
        cout << fullmpcdata.xlip_sol[4*i+2];
        cout << " " << fullmpcdata.xlip_sol[4*i+3] << endl;
    }
    */


    return 0;
}

