function slip = slip(radius, omega, vel) % radius, angular velocity, car velocity
    slip = (radius*omega - vel)/(vel^2);
end
