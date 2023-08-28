
// KY039 defines
#define TAB_LENGTH 4
#define RISE_THRESHOLD 5
#define CALIB_OFFSET 0


void setup() {
Serial.begin(115200);

}

void loop() {

float analog_Tab[TAB_LENGTH], analog_sum;
float last, analog_average, start;
float KY039_data, first, second, third, before;
bool rising;
int rise_count, n_reads;
long int last_beat, now, ptr;
// WiFi connection 
// Init variables
for (int i = 0; i < TAB_LENGTH; i++) analog_Tab[i] = 0;
analog_sum = 0;
ptr = 0;
while(1) {
// calculate an average of the sensor during a 20 ms period to eliminate the 50 Hz noise caused by electric light
n_reads = 0;
start = millis();
analog_average = 0.;
do {
analog_average += analogRead(A0);
n_reads++;
now = millis();
}
while (now < start + 20); 
analog_average /= n_reads; // we got an average
// Add the newest measurement to an array and subtract the oldest measurement from the array
// to maintain a sum of last measurements
analog_sum -= analog_Tab[ptr];
analog_sum += analog_average;
analog_Tab[ptr] = analog_average;
last = analog_sum / TAB_LENGTH;
// now last holds the average of the values in the array
// check for a rising curve (= a heart beat)
if (last > before) {
rise_count++;
if (!rising && rise_count > RISE_THRESHOLD) {
// we have detected a rising curve, which implies a heartbeat.
// Record the time since last beat, keep track of the two previous times (first, second, third) to get a weighed average.
// The rising flag prevents us from detecting the same rise more than once.
rising = true;
first = millis() - last_beat;
last_beat = millis();
// Calculate the weighed average of heartbeat rate according to the three last beats
KY039_data = 60000. / (0.4 * first + 0.3 * second + 0.3 * third)+CALIB_OFFSET;
Serial.print(KY039_data);
Serial.println(" BPM\n"); // Unit
third = second;
second = first;

}
}
else
{
// Ok, the curve is falling
rising = false;
rise_count = 0;
}
before = last;
ptr++;
ptr %= TAB_LENGTH;
}
}