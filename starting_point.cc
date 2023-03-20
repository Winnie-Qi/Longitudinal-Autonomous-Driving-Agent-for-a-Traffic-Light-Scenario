//
// Created by Gastone Pietro Rosati Papini on 10/08/22.
//

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <algorithm>

extern "C" {
#include "screen_print_c.h"
}
#include "screen_print.h"
#include "server_lib.h"
#include "logvars.h"

// --- MATLAB PRIMITIVES INCLUDE ---
// #include "primitives.h"
#include "pass_primitive.h"
#include "pass_primitive_j0.h"
#include "primitives_terminate.h"
#include "rt_nonfinite.h"
#include "stop_primitive.h"
#include "stop_primitive_j0.h"
// --- MATLAB PRIMITIVES INCLUDE ---

#define DEFAULT_SERVER_IP    "127.0.0.1"
#define SERVER_PORT                30000  // Server port
#define DT 0.05

// Handler for CTRL-C
#include <signal.h>
static uint32_t server_run = 1;
void intHandler(int signal) {
    server_run = 0;
}


#define CONTROL_DT 0.05
typedef struct PID_structure
{
    double Kp;
    double Ki;
    double integral;
    double out_max;
}PID_t;

double PID_calc(PID_t* pid, double error){
    double p,out;

    p = pid->Kp * error;
    pid->integral += pid->Ki * error;
    out = p +  pid->integral;

    if(out > pid->out_max)
    {
        out = pid->out_max;
        pid->integral = out - p;

    }
    else if (out < -pid->out_max)
    {
        out = -pid->out_max;
        pid->integral = out - p;
    }

    return out;
}

double coeff1[6], coeff2[6];
double T1,v1,T2,v2;
double sf, tf;

void FreeFlow(double v0, double a0, double lookhead, double vr, double **m)
{
    pass_primitive(v0,a0,lookhead,vr,vr,0,0,
                   coeff2, &v2, &T2,
                   coeff1, &v1, &T1);
    *m = coeff1;
}

void Pass(double v0, double a0, double sf, double vmin, double vmax, double Tmin, double Tmax, double **m1, double **m2)
{
    pass_primitive(v0,a0,sf,vmin,vmax,Tmin,Tmax,
                   coeff2, &v2, &T2,
                   coeff1, &v1, &T1);

    if( coeff1[0]==0.0 && coeff1[1]==0.0 && coeff1[2]==0.0 && coeff1[3]==0.0 && coeff1[4]==0.0 && coeff1[5]==0.0)
    {
        *m1 = nullptr;
        *m2 = nullptr;
    }
    else
    {
        *m1 = coeff1;   //1 2 order no difference
        *m2 = coeff2;
    }
}

void PassJ0(double v0, double a0, double xtr, double vmin, double vmax, double **m)
{
    pass_primitive_j0(v0, a0, xtr, vmin, vmax, coeff1, &sf, &tf);
    *m = coeff1;
}

void Stop(double v0, double a0, double xstop, double **m)
{
    stop_primitive(v0, a0, xstop, coeff1, &sf, &tf);
    *m = coeff1;
}

using namespace std;
int main(int argc, const char * argv[]) {
    logger.enable(true);

    // Messages variables
    scenario_msg_t scenario_msg;
    manoeuvre_msg_t manoeuvre_msg;
    size_t scenario_msg_size = sizeof(scenario_msg.data_buffer);
    size_t manoeuvre_msg_size = sizeof(manoeuvre_msg.data_buffer);
    uint32_t message_id = 0;

#ifndef _MSC_VER
    // More portable way of supporting signals on UNIX
//    struct sigaction act;
//    act.sa_handler = intHandler;
//    sigaction(SIGINT, &act, NULL);
#else
    signal(SIGINT, intHandler);
#endif

    server_agent_init(DEFAULT_SERVER_IP, SERVER_PORT);

    // Start server of the Agent
    printLine();
    printTable("Waiting for scenario message...", 0);
    printLine();

    //user vars
    bool is_first = true;
    double a_req_last = 0;
    PID_t pid_long;
    pid_long.Kp = 5;
    pid_long.Ki = 2;
    pid_long.integral = 0.0;
    pid_long.out_max = 30.0;

    double vmin = 3.0;
    double vmax = 15.0;
    double xs = 5.0;
    double xin = 10.0;

    double Ts = xs/vmin;
    double Tin = xin/vmin;
    double v0, a0, vr, lookhead;
    double Tgreen, Tred;
    double xtr = 0, xstop = 0;

    double *m1, *m2, *m;

    while (server_run == 1) {
        // Clean the buffer
        memset(scenario_msg.data_buffer, '\0', scenario_msg_size);

        // Receive scenario message from the environment
        if (server_receive_from_client(&server_run, &message_id, &scenario_msg.data_struct) == 0) {
            // Init time
            static auto start = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::now()-start;
            double num_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(time).count()/1000.0;
            //printLogTitle(message_id, "received message");

            // Data struct
            input_data_str *in = &scenario_msg.data_struct;
            manoeuvre_msg.data_struct.CycleNumber = in->CycleNumber;
            manoeuvre_msg.data_struct.Status = in->Status;

            // Example of using log
            //logger.log_var("Example", "cycle", in->CycleNumber);

            // ADD AGENT CODE HERE
            v0 = in->VLgtFild;
            a0 = in->ALgtFild;
            lookhead = max<double>(50, v0*5);
            vr = in->RequestedCruisingSpeed;
            printf("%d: TrfNr:%d, State:%d, Dist:%.2f ", in->CycleNumber, in->NrTrfLights, in->TrfLightCurrState, in->TrfLightDist);

            if(in->NrTrfLights != 0)
            {
                xtr = in->TrfLightDist;
                xstop = in->TrfLightDist - xs/2;
            }
            if(in->NrTrfLights == 0 || xtr >= lookhead)
            {
                FreeFlow(v0, a0, lookhead, vr, &m);
                printf("FreeFlow ");
            }
            else
            {
                switch(in->TrfLightCurrState)
                {
                    case 1: /* green */
                        Tgreen = 0.0;
                        Tred = in->TrfLightFirstTimeToChange - Tin;
                        break;
                    case 2: /* Yellow */
                        Tgreen = in->TrfLightSecondTimeToChange + Ts;
                        Tred = in->TrfLightThirdTimeToChange - Tin;
                        break;
                    case 3: /* Red */
                        Tgreen = in->TrfLightFirstTimeToChange + Ts;
                        Tred = in->TrfLightSecondTimeToChange - Tin;
                        break;
                    default:break; //TODO: safety?
                }

                if(in->TrfLightCurrState == 1 && in->TrfLightDist <= xs)
                {
                    FreeFlow(v0, a0, lookhead, vr, &m);
                    printf("FreeFlow ");
                }
                else
                {
                    Pass(v0,a0,xtr,vmin,vmax,Tgreen,Tred, &m1, &m2);
                    if( m1 == nullptr && m2 == nullptr)
                    {
                        Stop(v0, a0, xstop, &m);
                        printf("Stop ");
                    }
                    else
                    {
                        if( (m1[3]<0 && m2[3]>0) || (m1[3]>0 && m2[3]<0) )
                        {
                            PassJ0( v0, a0, xtr, vmin, vmax, &m);
                            printf("PassJ0 ");
                        }
                        else
                        {
                            if(abs(m1[3]) < abs(m2[3]))
                                m = m1;
                            else
                                m = m2;
                            printf("Pass ");
                        }
                    }
                }
            }

            // ADD LOW LEVEL CONTROL CODE HERE
            double deltaT = CONTROL_DT;
            double j_time0 = m[3];
            double j_timet = m[3] + m[4]*deltaT + 0.5*m[5]*deltaT*deltaT;
            double a_req_now = a0 + 0.5*(j_time0+j_timet)*deltaT;

            double padel_acc = PID_calc(&pid_long, a_req_now - a0);
            manoeuvre_msg.data_struct.RequestedAcc = padel_acc;

            printf("\n  v0: %f, a_req: %f, a0: %f, pid: %f, coeff: %f,%f,%f,%f,%f\n", v0, a_req_now, a0, padel_acc,
                                                                                                m[1],m[2],m[3],m[4],m[5]);
//            printf("previous a_req: %f, a0: %f\n", a_req_last, a0);
//            printf("current a_req: %f, pid: %f\n",a_req_now, padel_acc);
//            printf("driver requested cruising speed: %f m/s\n", vr);
//            printf("v0:%f, a0:%f, lookhead:%f\n", v0,a0,lookhead);
//            printf("caculated coeff1: %f, %f, %f, %f, %f, %f\n", coeff1[0],coeff1[1],coeff1[2],coeff1[3],coeff1[4],coeff1[5]);
//            printf("coeff1 optimal v: %f, optimal t: %f\n", v1, T1 );
//            printf("caculated coeff2: %f, %f, %f, %f, %f, %f\n", coeff2[0],coeff2[1],coeff2[2],coeff2[3],coeff2[4],coeff2[5]);
//            printf("coeff2 optimal v: %f, optimal t: %f\n", v2, T2 );

            // Write log
            //logger.write_line("Example");
            // Screen print
//            printLogVar(message_id, "Time", num_seconds);
//            printLogVar(message_id, "Status", in->Status);
//            printLogVar(message_id, "CycleNumber", in->CycleNumber);

            // Send manoeuvre message to the environment
            if (server_send_to_client(server_run, message_id, &manoeuvre_msg.data_struct) == -1) {
                perror("error send_message()");
                exit(EXIT_FAILURE);
            } else {
                //printLogTitle(message_id, "sent message");
            }
        }
    }

    // Close the server of the agent
    server_agent_close();
    return 0;
}