package config.neds;

import artery.envmod.Car;

module FaultyCar extends Car {
    parameters:
        @display("t=Faulty;i=node/car,red");
        middleware.services = default(xmldoc("config/faulty_services_cpm.xml"));
}
