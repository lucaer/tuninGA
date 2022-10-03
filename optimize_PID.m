function cost = optimize_PID(k)
assignin('base','k',k);
sim('PID_Motor.slx');
cost = ITAE(length(ITAE));
end