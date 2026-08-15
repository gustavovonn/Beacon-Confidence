package src.modules;

import artery.inet.Car;

module FaultyCar extends Car {
    parameters:
        @display("t=Byzantine;i=node/car,red");
        middleware.services = xmldoc("config/faulty_services.xml");
}
