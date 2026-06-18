#include <stdio.h>
#include "osal.h"
#include "ethercat.h"


char IOmap[4096];
boolean isAllSlaveInOpMode = FALSE;
int expectedWKC;
volatile int wkc;


/* most basic RT thread for process data, just does IO transfer */
void CALLBACK ThreadRealtimeTick(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{
    wkc = ec_receive_processdata(EC_TIMEOUTRET);
    ec_slave[1].outputs[0] = ec_slave[1].inputs[0];
    ec_send_processdata();
}


void SimpleTest(char* _ifname)
{
    const int CYCLE_TIME_NS = 1000000;

    UINT handleRealtimeTick;
    printf("Starting simple test...\n");

    /* initialise SOEM, bind socket to _ifname */
    if(ec_init(_ifname))
    {
        printf("ec_init on %s succeeded.\n", _ifname);

        /* find and auto-config slaves */
        if(ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            ec_config_map(&IOmap);

            ec_configdc();
            // IMPORTANT TO ENABLE DC
            for(int s = 1; s <= ec_slavecount; s++)
                ec_dcsync0(s, TRUE, CYCLE_TIME_NS, 0);

            printf("Slaves mapped, state to SAFE_OP.\n");

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

            /* start RT thread as periodic MM timer */
            handleRealtimeTick = timeSetEvent(1, 0, ThreadRealtimeTick, 0, TIME_PERIODIC);

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

            /* request OP state for all slaves */
            // use slave id-0 for broadcast
            printf("Request operational state for all slaves...\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_writestate(0);

            /* wait for all slaves to reach OP state */
            int chk = 200;
            do
            {
                ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
            } while(chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if(ec_slave[0].state == EC_STATE_OPERATIONAL)
            {
                printf("OP state reached for all slaves.\n");
                isAllSlaveInOpMode = TRUE;

                /* cyclic loop, reads data from RT thread */
                for(int i = 1; i <= 500000000; i++)
                {
                    printf(" T:%lld wkc:%d\r", ec_DCtime, wkc);

                    osal_usleep(100000);
                }
                isAllSlaveInOpMode = FALSE;
            } else
            {
                printf("Not all slaves reached operational state.\n");

                ec_readstate();
                for(int i = 1; i <= ec_slavecount; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                               i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                               ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            /* stop RT thread */
            timeKillEvent(handleRealtimeTick);

            printf("\nRequest init state for all slaves\n");

            /* request INIT state for all slaves */
            ec_slave[0].state = EC_STATE_INIT;
            ec_writestate(0);
        } else
        {
            printf("No slaves found!\n");
        }

        /* stop SOEM, close socket */
        printf("End simple test, close socket\n");
        ec_close();
    } else
    {
        printf("No socket connection on %s\nExcecute as root\n", _ifname);
    }
}


OSAL_THREAD_FUNC ThreadEcatStateCheck(void* lpParam)
{
    int groupId = 0;

    while(TRUE)
    {
        if(isAllSlaveInOpMode && ((wkc < expectedWKC) || ec_group[groupId].docheckstate))
        {
            /* one ore more slaves are not responding */
            ec_group[groupId].docheckstate = FALSE;
            ec_readstate();

            for(int i = 1; i <= ec_slavecount; i++)
            {
                if((ec_slave[i].group == groupId) && (ec_slave[i].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[groupId].docheckstate = TRUE;
                    if(ec_slave[i].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", i);
                        ec_slave[i].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(i);
                    } else if(ec_slave[i].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", i);
                        ec_slave[i].state = EC_STATE_OPERATIONAL;
                        ec_writestate(i);
                    } else if(ec_slave[i].state > EC_STATE_NONE)
                    {
                        if(ec_reconfig_slave(i, EC_TIMEOUTRET3))
                        {
                            ec_slave[i].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", i);
                        }
                    } else if(!ec_slave[i].islost)
                    {
                        /* re-check state */
                        ec_statecheck(i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET3);
                        if(ec_slave[i].state == EC_STATE_NONE)
                        {
                            ec_slave[i].islost = TRUE;
                            printf("ERROR : slave %d lost!\n", i);
                        }
                    }
                }

                if(ec_slave[i].islost)
                {
                    if(ec_slave[i].state == EC_STATE_NONE)
                    {
                        if(ec_recover_slave(i, EC_TIMEOUTRET3))
                        {
                            ec_slave[i].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", i);
                        }
                    } else
                    {
                        ec_slave[i].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", i);
                    }
                }
            }

            if(!ec_group[groupId].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }

        osal_usleep(100000);
    }
}


int main(int argc, char* argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if(argc > 1)
    {
        /* create thread to handle slave error handling in OP */
        OSAL_THREAD_HANDLE handleErrorHandler;
        osal_thread_create(&handleErrorHandler, 128000, &ThreadEcatStateCheck, (void*) &ctime);

        /* start cyclic part */
        SimpleTest(argv[1]);
    } else
    {
        printf("Usage: simple_test ifname1\n");
        /* Print the list */
        printf("Available adapters\n");
        ec_adaptert* adapters = ec_find_adapters();

        while(adapters != NULL)
        {
            printf("Description : %s, Device to use for wpcap: %s\n", adapters->desc, adapters->name);
            adapters = adapters->next;
        }
    }

    printf("End program\n");
    return 0;
}
