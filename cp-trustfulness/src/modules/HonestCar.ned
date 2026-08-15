package src.modules;

import artery.inet.Car;

module HonestCar extends Car {
    parameters:
        @display("t=Honest;i=node/car,green");
        middleware.services = xmldoc("config/services.xml");
}
