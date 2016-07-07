// This line-follow uses PID feedback control

#include "mbed.h"
#include "m3pi_ng.h"
#include <ctime> //added
#include <cassert>
#include <cmath>
#include "btbee.h"
#include <vector>
#include <stdlib.h>
#include "MSCFileSystem.h"
#include <fstream>

MSCFileSystem msc("usb"); // Mount flash drive under the name "msc"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// ROBOT FUNCTIONS ////////////////////////////////////////////////////////////////////////

// EFFECTS: Displays the voltage of the battery on the m3pi screen
void disp_battery(m3pi &pi){
    float volt = pi.battery();
    pi.cls();
    char array[10];
    sprintf(array, "%f", volt); 
    pi.print(array, 5);
}

// EFFECTS: Calibrates the sensor initially
void calibrate_sensor(m3pi &pi){
    pi.sensor_auto_calibrate();
    int sens[5];
    pi.calibrated_sensor(sens);
    /*for ( int count = 0; count < 5; ++count) {
        assert (sens[count]);
        }*/
}

// EFFECTS: Returns true if sensors detect the black line, false if they do not
bool check_sensors(m3pi &pi, int SENSOR_THRESHOLD){
    int sensor[5];
    pi.calibrated_sensor(sensor);
    for(int i = 0; i < 5; ++i){
        if (sensor[i] < SENSOR_THRESHOLD){
            return false;
        }
    }
    return true;
}
        
double run_robot(double constant_p, double constant_d, m3pi &pi, ofstream &myFile, FILE *fp) {
    DigitalIn button(p21);
    button.mode(PullUp);
    
    // Initialize the robot and btbee
    // m3pi pi;
    // btbee input;
    
    const float MAX_SPEED = 1.0;
    const float MIN_SPEED = 0.0;

    float speed = 0.6;

    // float k_p = 0.6, k_d = 0.6
    float k_i = 0.0;
    float k_p = float(constant_p);
    float k_d = float(constant_d);
    float p_correction = 0.0;
    float i_correction = 0.0;
    float d_correction = 0.0;
    float tot_correction = 0.0;
    float right_speed = 0.0;
    float left_speed = 0.0;
    float previous_position = 0.0;
    float current_position = 0.0;
    float elapsed_secs;
    int lap_count = 0;
    int sensor[5];

    pi.reset();

    // Find and display the battery voltage
    pi.cls();
    disp_battery(pi);
    wait(2);

    // Calibrate the sensors
    calibrate_sensor(pi);
    
    // Move the robot to the beginning of the line
    float r_speed = 0.3;
    float r_kp = 0.3;
    float r_pos;
    float r_right_speed;
    float r_left_speed;
    while(1){
        if (check_sensors(pi, 350)){
            break;
        }
        else{
            r_pos = pi.line_position();
            r_right_speed = r_speed + r_kp * r_pos;
            r_left_speed = r_speed - r_kp * r_pos;
            
            pi.right_motor(r_right_speed);
            pi.left_motor(r_left_speed);   
        }
    }
    pi.stop();
    wait(3);

    bool run_condition = true;
    
    clock_t begin = clock();
    clock_t lap_clock = clock();

    while (run_condition) {
        // If error button is pressed, pause the robot and return a dramatically high time value
        if(!button){
            pi.stop();
            wait(6);
            return 1000.0;
        }
        
        //IMPLEMENT FEEDBACK CONTROLS HERE--------------------------------------------------------------->
        current_position = pi.line_position();
    
        // P-Control
        p_correction = current_position;
        
        // D-Control
        d_correction = current_position - previous_position;
        
        // I-Control
        i_correction += current_position;
         
        // Total Correction
        tot_correction = (k_p * p_correction) + (k_i * i_correction) + (k_d * d_correction);
        
        // Remember the last position
        previous_position = current_position;
        //---------------------------------------------------------------------------------------------->
        
        // Establish the speeds for the right/left motors. May need to switch the operations.
        right_speed = speed + tot_correction;
        left_speed = speed - tot_correction;
        
        // Apply Checks
        if (right_speed > MAX_SPEED){
            right_speed = MAX_SPEED;
        }
        else if(right_speed < MIN_SPEED){
            right_speed = MIN_SPEED;
        }
        
        if (left_speed > MAX_SPEED){
            left_speed = MAX_SPEED;
        }
        else if (left_speed < MIN_SPEED){
            left_speed = MIN_SPEED;
        }
        
        // Drive the motors
        pi.right_motor(right_speed);
        pi.left_motor(left_speed);

        // Check the sensors for counting laps
        pi.calibrated_sensor(sensor);
        
        if (sensor[0] >= 350 && sensor[1] >= 350 && sensor[2] >= 350 && sensor[3] >= 350 && sensor[4] >= 350){
            
            if (((clock() - lap_clock) / CLOCKS_PER_SEC) > 4.0){
                lap_count++;
                //lap_time = (lap_clock - clock())/CLOCKS_PER_SEC;
                lap_clock = clock();
            }
        }

        if (lap_count == 2) { 
            clock_t end = clock(); // added
            elapsed_secs = (end - begin); //added
            pi.cls();
            char arrays[10];
            sprintf(arrays, "%f", elapsed_secs/CLOCKS_PER_SEC);
            pi.print(arrays, 8); //added
            pi.stop();
            wait(5);
            //pi.printf("Time: %f || K_p: %f || K_d: %f || K_i: %f \n", elapsed_secs/CLOCKS_PER_SEC, k_p, k_d, k_i); 
            run_condition = false;
            break;
        }
        //begin = clock() - begin; //reset clock for the loop
        
    }
    // pi.stop();
    myFile << "Kp: " << k_p << "|| Kd: " << k_d << "|| Time: " << elapsed_secs/CLOCKS_PER_SEC << endl;
    // double time = elapsed_secs /CLOCKS_PER_SEC; 
    fprintf(fp, "Kp: %f || Kd %f || %f \n", k_p, k_d, elapsed_secs/CLOCKS_PER_SEC);
    return (elapsed_secs / CLOCKS_PER_SEC);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// NELDER_MEAD FUNCTIONS //////////////////////////////////////////////////////////////


// EFFECTS: Sorts the times vector from fastest to slowest time
void time_sort(vector<vector<double> > &time){
    vector<double> tmp(2);
    int j;
    for(int i = 1, size = time.size(); i < size; ++i){
        j = i;
        tmp = time[i];
        while (j > 0 && (tmp[0] < time[j-1][0])){
            time[j] = time[j - 1];
            j--;
        }
        time[j] = tmp;
    }
}

// EFFECTS: Finds the centroid of all the points but the worst point
void find_centroid(vector<vector<double> > &time, vector<vector<double> > &vertices, vector<double> &centroid){
    double sum = 0;
    int num_elements = 0;
    // Iterate through each dimension
    for(int i = 0, size = vertices[i].size(); i < size; ++i){
        // Iterate through each vertex except the last one
        for(int j = 0, size2 = time.size() - 1; j < size2; ++j){
            sum += vertices[time[j][1]][i];
            num_elements += 1;
        }
        centroid[i] = sum/num_elements;
        num_elements = 0;
        sum = 0;
    }
}

// EFFECTS: Transforms the simplex by reflecting the worst point across the centroid
void reflect(vector<double> &transform_coords, vector<double> &centroid, vector<double> &worst, int num_dims){
    for(int i = 0; i < num_dims; ++i){
        transform_coords[i] = 2 * centroid[i] - worst[i];
    }
}

// EFFECTS: Transforms the simplex by expanding beyond the reflection point and storing it in transform_coords
void expand(vector<double> &transform_coords, vector<double> &centroid, vector<double> &reflected, int num_dims){
    for(int i = 0; i < num_dims; ++i){
        transform_coords[i] = 2 * reflected[i] - centroid[i];
    }
}

// EFFECTS: Transforms the simplex by finding the midpoint between 2 points and storing them in the transform_coords
void contraction(vector<double> &transform_coords, vector<double> &A, vector<double> &B, int num_dims ){
    for(int i = 0; i < num_dims; ++i){
        transform_coords[i] = (A[i] + B[i]) / 2;
    }
}

// EFFECTS: Transforms the simplex by shrinking every vertex besides the best vertex towards the best vertex
// via midpoint formula
void shrink(vector<vector<double> > &vertices){
    for (int i = 1, size = vertices.size(); i < size; ++i){
        for (int j = 0, size2 = vertices[i].size(); j < size2; ++j){
            vertices[i][j] = (vertices[i][j] + vertices[0][j]) / 2;
        }
    }
}

// EFFECTS: Checks to see if the function values (robot times) fall within a certain precision
// amount between each other. If they do, return true. Else return false
bool check_precision(vector<vector<double> > &time, double precision){
    for(int i = 0, size = time.size() - 1; i < size; ++i){
        for(int j = i + 1, size2 = time.size(); j < size2; ++j){
            if (fabs(time[i][0] - time[j][0]) > precision){
                return false;
            }
        }
    }
    return true;
}

// EFFECTS: Replaces the first n elements of the first vector with the first n elements of the second vector
void replace_dims(vector<double> &vec1, vector<double> &vec2, int n){
    // cout << "Running replace_dims" << endl;
    for(int i = 0; i < n; ++i){
        vec1[i] = vec2[i];
    }
}

// EFFECTS: Computes the optimal parameter values for a function within the bounds
// of a certain threshld.
// initial_vertices: a vector of vectors that stores n + 1 vertices of n parameters
void nelder_mead(vector<vector<double> > &vertices, double precision, double (*run_robot)(double, double, m3pi &, ofstream &, FILE *), int num_dims, m3pi &pi, ofstream &myFile, FILE *fp /*, vector<double> &solution*/ ){

    // Keep track of the number of times the robot runs and the number of iterations of the algorithm
    int num_iterations = 0;
    int num_runs = 0;

    // Storage variable to keep track of a tested time for comparison with the currently tested times
    double test_time = 0;

    // Keep track of centroid
    vector<double> centroid(vertices[0]);

    // Establish vector of vectors for testing vertices at Reflection, Expansion, Contraction1, Contraction2
    // <dim1, dim2, dim3, time>
    // [0] = reflection data
    // [1] = expansion data
    // [2] = contraction1 data
    // [3] = contraction2 data
    vector<double> test_setup(vertices[0].size() + 1, 0);
    vector<vector<double> > test(4, test_setup);

    // Initialize the vector of vectors of times and the index of the vertex that corresponds to it
    // <time, index_of_vertex>
    // <time, index_of_vertex>
    // ...
    vector<double> time_setup (2, 0);
    vector<vector<double> > time(vertices.size(), time_setup);

    // Initially find the times of the vertices
    for(int i = 0, size = vertices.size(); i < size; ++i){
        time[i][0] = run_robot(vertices[i][0], vertices[i][1], pi, myFile, fp);
        time[i][1] = i;
        num_runs++;
    }

    // Sort the vertices into best, good, and worst indices
    time_sort(time);
    // Find the first centroid and save it into the centroid vector
    find_centroid(time, vertices, centroid);

    // Run the robot at the reflection point and make a comparison with the best time
    reflect(test[0], centroid, vertices[time.back()[1]], num_dims);
    test[0].back() = run_robot(test[0][0], test[0][1], pi, myFile, fp); 
    num_runs++;

    test_time = test[0].back();

    // while(!check_precision(time, precision)){
    while(num_iterations < 12){

        // cout << "test_time >= time[0][0]: " << (test_time >= time[0][0]) << endl;
        // cout << "test_time < time[time.size() - 2][0]: " << (test_time < time[time.size() - 2][0]) << endl;
        // cout << "Time.back()[1]: " << time.back()[1] << endl;
        // If Reflection is faster than second Worst but slower than or equal to the best, and 
        // the difference is more than the precision, replace Worst with Reflection in the simplex
        if (((test_time >= time[0][0]) || ((test_time < time[0][0]) && (fabs(test_time - time[0][0]) < precision))) && test_time < time[time.size() - 2][0] /*&& fabs(test_time - time[0][0]) <= precision*/){
            replace_dims(vertices[time.back()[1]], test[0], num_dims);
            time.back()[0] = test_time;
        }
        // If Reflection is better than the current best, expand using the reflection point,
        // run the robot, and replace Worst with the better of reflected and expanded
        else if(test_time < time[0][0] && fabs(test_time - time[0][0]) > precision){
            expand(test[1], centroid, test[0], num_dims);
            test[1].back() = run_robot(test[1][0], test[1][1], pi, myFile, fp);
            num_runs++;

            if(test[1].back() < test[0].back()){
                // Update vertices with expanded data
                replace_dims(vertices[time.back()[1]], test[1], num_dims);
                time.back()[0] = test[1].back();
                // cout << "using E 2" << endl;
            }
            else{
                // Update vertices with reflected data
                // cout << "using R 1" << endl;
                // print_test(test, 0);
                // print_test(vertices, time.back()[1]);

                replace_dims(vertices[time.back()[1]], test[0], num_dims);
                // cout << "Made it " << endl;
                time.back()[0] = test[0].back();
                // cout << "using R 2" << endl;
            }
        }
        // If Reflection is better than the worst but slower than the second worst, apply contractions
        else if(test_time < time.back()[0] && test_time > time[time.size() - 2][0]) {
            contraction(test[2], centroid, test[0], num_dims);
            test[2].back() = run_robot(test[2][0], test[2][1], pi, myFile, fp);
            num_runs++;

            contraction(test[3], centroid, vertices[time.back()[1]], num_dims);
            test[3].back() = run_robot(test[3][0], test[3][1], pi, myFile, fp);
            num_runs++;

            // Compare the two contractions and replace worst with the best contraction
            if(test[2].back() < test[3].back() && fabs(test[2].back() - test[3].back()) > precision){
                replace_dims(vertices[time.back()[1]], test[2], num_dims);
                time.back()[0] = test[2].back();
            }
            else{
                replace_dims(vertices[time.back()[1]], test[3], num_dims);
                time.back()[0] = test[3].back();
            }
        }
         // If Reflection is worse than the current worst, apply shrinks and run the robot with the new shrinked values
        else{
            // assert(test_time > time.back()[0]);
            shrink(vertices);
            for(int i = 1, size = time.size(); i < size; ++i){
                time[i][0] = run_robot(vertices[i][0], vertices[i][1], pi, myFile, fp);
                time[i][1] = i;
                num_runs++;
            }
        }
        // After each iteration, re-sort the times, recompute centroid, reflect the worst point, and run_robot to find the new test_time
        time_sort(time);
        find_centroid(time, vertices, centroid);
        reflect(test[0], centroid, vertices[time.back()[1]], num_dims);
        test[0].back() = run_robot(test[0][0], test[0][1], pi, myFile, fp);
        num_runs++;
        test_time = test[0].back();

        // Write the current best vertices to solution
        // for(int i = 0; i < num_dims; ++i){
        //     solution[i] = vertices[time[0][1]][i];
        // }
        // solution[2] = time[0][0];

        num_iterations++;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// RUNNING AND TESTING //////////////////////////////////////////////////////////////////


void test_1(m3pi &pi, ofstream &myFile, FILE *fp){

    vector<double> vertex_1(2);
    vertex_1[0] = 0.6; vertex_1[1] = 0.6;

    vector<double> vertex_2(2);
    vertex_2[0] = 0.7; vertex_2[1] = 0.85;

    vector<double> vertex_3(2);
    vertex_3[0] = 0.5; vertex_3[1] = 0.7;

    vector<vector<double> > vertices;
    vertices.push_back(vertex_1);
    vertices.push_back(vertex_2);
    vertices.push_back(vertex_3);

    // cout << "HERE START" << endl;
    nelder_mead(vertices, 0.0001, run_robot, 2, pi, myFile, fp);
    // for(int i = 0, size = solution.size(); i < size; ++i){
    //     cout << solution[i] << endl;
    // }
}

void test_2(m3pi &pi, ofstream &myFile, FILE *fp){
    double time1 = run_robot(0.6, 0.6, pi, myFile, fp);
    wait(10);
    double time2 = run_robot(2.11, 8.71, pi, myFile, fp);
}

int main(){
    FILE *fp = fopen("/usb/test.txt", "w");
    ofstream myFile ("/usb/data.txt"); 
    m3pi pi;
    test_1(pi, myFile, fp);
    //test_2(pi, myFile, fp);
    myFile.close();
    fclose(fp);
    return 0;
}
