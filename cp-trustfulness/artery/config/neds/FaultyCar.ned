package config.neds;

import artery.inet.Car;

module FaultyCar extends Car {
    parameters:
        @display("t=Faulty;i=node/car,red");
        middleware.services = xmldoc("config/faulty_services.xml");
}
