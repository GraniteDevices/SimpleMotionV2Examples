#include <stdio.h>
#include <unistd.h> //for usleep
#include "simplemotion.h"

smbus handle;
int nodeAddress;

bool setupScope()
{
    SM_STATUS status = 0;

    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_BUF_LENGHT, 2048);
    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_BEFORE_TRIGGER_PERCENTS, 0); //note, requires on IONI FW 1.6.0 or later
    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_SAMPLERATE, 7); //with IONI sample rate is 20000/(SMP_CAPTURE_SAMPLERATE+1) Hz, so here it is 2500 Hz
    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_SOURCE, BV(CAPTURE_POSITION_ACTUAL)|BV(CAPTURE_BUS_VOLTAGE));
    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_TRIGGER, TRIG_INSTANT);
    status |= smSetParameter(handle, nodeAddress, SMP_CAPTURE_STATE, 1); //start capture

    if(status!=SM_OK)
    {
        //error
        return false;
    }
    return true;
}

//parameters: nsamples=number of samples to read 1-2048, samples=pointer to buffer with at least nsamples length
bool downloadScope( int nsamples, int *samples )
{
    smint32 retval;
    int samplesread=0, storepos=0;
    SM_STATUS smStat=0;

    //setup for scope reading
    smStat|=smAppendSMCommandToQueue( handle, SMPCMD_SET_PARAM_ADDR, SMP_RETURN_PARAM_LEN );
    smStat|=smAppendSMCommandToQueue( handle, SMPCMD_24B, SMPRET_32B );//read 32 bit (values capped to 30 bits) samples
    smStat|=smAppendSMCommandToQueue( handle, SMPCMD_SET_PARAM_ADDR, SMP_RETURN_PARAM_ADDR );
    smStat|=smAppendSMCommandToQueue( handle, SMPCMD_24B, SMP_CAPTURE_BUFFER_GET_VALUE ); //read at get_value param
    smStat|=smAppendSMCommandToQueue( handle, SMPCMD_SET_PARAM_ADDR, SMP_CAPTURE_BUFFER_GET_ADDR ); //set store address to get_addr
    smStat|=smExecuteCommandQueue(handle,nodeAddress);
    smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );
    smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );
    smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );
    smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );
    smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );

    //loop to read samples
    while(samplesread<nsamples)
    {
        int i;
        int samplestofetch=nsamples-samplesread;
        if(samplestofetch>30)samplestofetch=30;//maximum per one SM cycle (4*30=120 bytes=max payload)

        //add read param commands to queue
        for(i=0;i<samplestofetch;i++)
        {
            smStat|=smAppendSMCommandToQueue( handle, SMPCMD_24B, samplesread );
            samplesread++;
        }

        //transmit & redeive over SM bus
        smStat|=smExecuteCommandQueue(handle,nodeAddress);

        //read values from return data queue
        for(i=0;i<samplestofetch;i++)
        {
            smStat|=smGetQueuedSMCommandReturnValue(  handle, &retval );
            samples[storepos++]=retval;
        }
    }

    //read one dummy variable just to cause SMP_RETURN_PARAM_ADDR to change to non-SMP_CAPTURE_BUFFER_GET_VALUE so next time we read data, we get it all from beginning
    smStat |= smRead1Parameter(handle, nodeAddress, SMP_NULL, &retval );

    if(smStat!=SM_OK)
        return false;

    return true;
}

SM_STATUS scopeWait()
{
    smint32 state;

    //loop until SMP_CAPTURE_STATE is 0 (scope idle)
    while (true)
    {
        SM_STATUS status = smRead1Parameter(handle, nodeAddress, SMP_CAPTURE_STATE, &state);

        if (status != SM_OK)
            return status;

        fprintf(stderr, "scope state: %d\n", state);

        if (state == 0)
        {
            break;
        }

        usleep(100000);
    }

    return SM_OK;
}


int main( void )
{
    const char *portName="COM3"; //bus device name
    nodeAddress = 4; //SM device address
    int samples[2048];

    fprintf(stderr, "opening bus %s\n", portName);
    handle = smOpenBus(portName);

    if (handle < 0)
    {
        fprintf(stderr, "could not open bus: %s\n", portName);
        return -4;
    }

    if(setupScope()==false)
        return -1;

    if(scopeWait()!=SM_OK)
        return -2;

    if(downloadScope( 2048, samples )==false)
        return -3;

    //sample list will be repeating list of samples defined to SMP_CAPTURE_SOURCE.
    //I.e if we capture 3 channels A, B and C, then samples will contain A0,B0,C0,A1,B1,C1,A2,B3,C3 etc
    for( int i=0; i<2048; i++)
    {
        fprintf(stderr, "samples[%d]=%d\n", i, samples[i]);
    }

    fprintf(stderr, "All done\n");
    return 0;
}
