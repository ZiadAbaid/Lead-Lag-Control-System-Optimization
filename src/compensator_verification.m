orig_sys = tf ([658.56] , [1,56,3136,0,0]) ; 
comp = tf([1,1.6] , [1,7]) * tf([1,0.0975] , [1,0.005]) ; 

full_sys = orig_sys * comp ; 

feedback_sys =feedback(full_sys , 1) 

