# Medication Dispenser MCU File



## Sending data from the Nextion Display to the Raspberry PI

The data which is being sent from the nextion display is formatted in UTF8.
The data is being transferred in JSON language.
Here is the struct which gets used for transferring data:\
```
{
  "page": <number of page/screen from HMI Display:int>,
  "action": "PushButton,ButtonRelease,Exit,Fail, you can take whatever you want from here",
  "payload": "Whatever Payload you wanna wish the MCU wants to know"
}
```