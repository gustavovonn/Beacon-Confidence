import CollectivePerceptionService;

simple MonitorCollectivePerception extends CollectivePerceptionService
{
    parameters:
        bool smartdataEnabled = default(true);

    @class("MonitorCollectivePerception"); 
}
