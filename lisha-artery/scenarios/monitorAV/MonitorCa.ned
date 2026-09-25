import artery.application.CaService;

simple MonitorCa extends CaService
{
    
    parameters:
        bool smartdataEnabled = default(true);

    @class("MonitorCa");
}
