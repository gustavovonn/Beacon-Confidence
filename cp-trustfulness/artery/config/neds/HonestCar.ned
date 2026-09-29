package config.neds;

import artery.envmod.Car;

module HonestCar extends Car {
    parameters:
        @display("t=Honest;i=node/car,green");
        middleware.services = default(xmldoc("config/services_cpm.xml"));
}
