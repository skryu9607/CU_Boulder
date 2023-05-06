function u_t = ImpAndDblet(t,np,imp_start,imp_end,amp)
if np == 1
    u_t = amp * stepfun(t,imp_start);
    u_t = u_t - amp * stepfun(t,imp_end);
else
    u_t = amp * stepfun(t,imp_start(1));
    u_t = u_t - amp * stepfun(t,imp_end(1));
    u_t = u_t - amp * stepfun(t,imp_start(2));
    u_t = u_t + amp * stepfun(t,imp_end(2));
end

