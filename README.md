# PID Controller Project
The PID control algorithm is implemented as per Sebastian's slides. To start off with I actually took some inspiration from Sebastians values:
```
Kd = 4.0
Kp = 0.8
Ki = 0.002
```

I implemented a poor man's twiddle algorithm where the algo would sum errors over the last 'window' samples (this is currently at 100) and chanage the constants one after the other in real-time. This doesn't make too much sense to me since the most 'correct' way would be sum over the entire track but ain't nobody got time for that so its only looking at a window of 100 samples and hoping for the best. Any help here would be appreciated on how to make this better.

The car seems to drunkenly go over the entire track endlessly. Hoping the MPD controller is better than this :)

After some twiddling, I arrived at the following params and have set them as the starting point for the project:

```
  Kd = 5.2816
  Kp = 0.7
  Ki = 0.002
```

A video of this could be found here: https://youtu.be/QDYG9qImt9M
