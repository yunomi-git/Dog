import matplotlib.pyplot as plt 
import random
random.seed(1003)

# initial conditions
x0 = 0;
t0 = 0;

# final conditions
tf = 1000;
xf = 1000;

resolution = 1000; # number of times to measure

def updateSystem(x_current, t_current, x_final, t_final, t_prev):
    return x_current + (x_final - x_current) * (t_current - t_prev) / (t_final - t_prev);

history_accurate = [0] * (resolution + 1)
history_accurate[0] = x0;
history_perturbed = [0] * (resolution + 1)
history_accurate[0] = x0;

history_errors = [0] * (resolution + 1)

t_current_accurate = t0;
t_current_perturbed = t0;

dt = (tf - t0)/resolution;

for i in range(0, resolution):
    # save previous state
    t_prev_accurate = t_current_accurate
    t_prev_perturbed = t_current_perturbed
    
    # update current state
    t_current_accurate = t_current_accurate + dt;
    t_current_perturbed = t_current_accurate + 2*(random.random() - 0.5) * (tf - t0) / resolution
    # t_current_accurate =  t_current_accurate + 1 + 2*random.random()
    # t_current_perturbed = round(t_current_accurate)
    
    if (t_current_accurate > tf):
        break
        
    history_accurate[i+1] = updateSystem(history_accurate[i], t_current_accurate, 
                                         xf, tf, t_prev_accurate)
    history_perturbed[i+1] = updateSystem(history_perturbed[i], t_current_perturbed, 
                                          xf, tf, t_prev_perturbed)
    history_errors[i+1] = history_accurate[i+1] - history_perturbed[i+1]

plt.plot(range(0, resolution+1), history_errors);
plt.show();
plt.plot(range(0, resolution+1), history_accurate, label="accurate");
plt.plot(range(0, resolution+1), history_perturbed, label="perturbed");
plt.show()



    