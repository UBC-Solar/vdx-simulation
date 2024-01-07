function CoF = pacejka(slip, A, B, C, D, E)
    CoF = D*sin(C*atan(B*slip - (E*(B*slip - atan(B*slip)))));
end
