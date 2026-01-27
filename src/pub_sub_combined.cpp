// #include "PubSub.h"

// PubSub pubsub;
// bool led_on = false;

// void command_test()
// {
//     switch (linear_actuator_cmd)
//     {
//         case 1:
//         case -1:
//         case 0:
//         case 4:
//         case 8:
//         case 16:
//             led_on = true;
//             linear_actuator_cmd = -6;
//         default:
//             break;
//     }
//     switch (drill_cmd)
//     {
//         case -2:
//         case -1:
//         case 0:
//         case 1:
//         case 2:
//             led_on = true;
//             drill_cmd = -6;
//         default:
//             break;
//     }
//     switch (barrel_cmd)
//     {
//         case 1:
//         case 0:
//         case 4:
//         case 8:
//         case 16:
//             led_on = true;
//             barrel_cmd = -6;
//         default:
//             break;
//     }
//     switch (servo_toggle)
//     {
//         case 0:
//         case 1:
//         case 2:
//             led_on = true;
//             servo_toggle = -6;
//         default:
//             break;
//     }
//     switch (science_module_toggle)
//     {
//         case 0:
//         case 1:
//             led_on = true;
//             science_module_toggle = -6;
//         default:
//             break;
//     }

//     if (led_on)
//     {
//       digitalWrite(LED_BUILTIN, HIGH);
//       delay(500);
//       digitalWrite(LED_BUILTIN, LOW);
//       led_on = false;
//     }
// }

// void setup() {
//   Serial.begin(115200);
//   pinMode(LED_BUILTIN, OUTPUT);
//   pubsub.init();
// }

// void loop() {
//     pubsub.publish_drill_data(
//         false,
//         random(-250, 250),
//         random(-250, 250) / 10.0, random(-250, 250) / 10.0, random(-250, 250) / 10.0,
//         random(-250, 250) / 10.0, random(-250, 250) / 10.0, random(-250, 250) / 10.0
//     );

//     pubsub.publish_sensor_data(
//         true,   // colorless
//         false,  // purple
//         random(-250, 250) / 10.0,   // humidity
//         random(-250, 250), random(-250, 250), random(-250, 250), // NPK
//         random(-250, 250) / 10.0,    // pH
//         random(-250, 250),    // CO2
//         random(-250, 250) / 10.0,   // temp
//         random(-250, 250), // pressure
//         random(-250, 250) / 10.0,  // altitude
//         random(0, 250),   // depth
//         0, // latitude
//         0  // longitude
//     );


//   pubsub.publish_info_warning(random(-10, 10));

//   pubsub.handle_subscriptions();

//   pubsub.publish_cmd_received(
//       linear_actuator_cmd,
//       drill_cmd,
//       barrel_cmd,
//       servo_toggle,
//       science_module_toggle
//   );

//   command_test();
// }