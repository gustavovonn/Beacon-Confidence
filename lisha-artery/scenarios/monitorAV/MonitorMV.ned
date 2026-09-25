import artery.application.ItsG5Service;

simple MonitorMV like ItsG5Service
{
    parameters:
    bool fileRecord = default(true);
    bool smartdataEnabled = default(false);
    string iotURL = default("https://iot.lisha.ufsc.br");
    int sdUploadCount = default(1);
}
