function setPID(P, I, D)
    P = string(P);
    I = string(I);
    D = string(D);
    set_param("genSimu/generator PID/PID Controller", "P", P);
    set_param("genSimu/generator PID/PID Controller", "I", I);
    set_param("genSimu/generator PID/PID Controller", "D", D);
end

