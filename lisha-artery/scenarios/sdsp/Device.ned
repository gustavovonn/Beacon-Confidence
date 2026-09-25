

import artery.application.ItsG5Service;

simple Device like ItsG5Service
{
    parameters:
        @display("i=block/cogwheel");

    @signal[authenticationLatency](type=simtime_t);
    @signal[v2vLatency](type=simtime_t);
    @signal[ecdhLatency](type=simtime_t);
    @signal[otpLatency](type=simtime_t);
    @signal[aesEncryptLatency](type=simtime_t);
    @signal[aesDecryptLatency](type=simtime_t);
    @signal[groupKeyLatency](type=double);

    @statistic[ECDHLatencyTotal] (title="ecdhLatency Latency"; source = ecdhLatency; record = vector);
}
