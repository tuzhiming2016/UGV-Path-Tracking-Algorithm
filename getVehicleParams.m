function VehicleParams = getVehicleParams(Vehicle)
    if Vehicle == "C-Class-Hatchback"
        VehicleParams = load('VehicleParams_C_Class.mat');
        VehicleParams.Wmax=0.85*1.0*9.806/10;
        VehicleParams.Wmin=-0.85*1.0*9.806/10;
        VehicleParams.Vmax=150;
        VehicleParams.Vmin=-60;
    end