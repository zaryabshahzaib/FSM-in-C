## Introduction 
FSM.c implements a state machine that allows entry of text using a 4x4 keypad, similar to the techniques used in old phones. If you look at the keypads, each key has a sequence of alphabets printed alongside the main key number.

![Keypad](https://upload.wikimedia.org/wikipedia/commons/thumb/7/73/Telephone-keypad2.svg/1200px-Telephone-keypad2.svg.png)

Number 2 contains ABC, Number 3 contains DEF, and so on. In order to enter B, I
can press key 2 twice, in quick succession. If I need A, I can press 2 once and then
pause for a little bit for the system to register that I wanted A.

## Functionality
Implemented a state machine that replicates the behavior of text entry on old
phones: If a key is pressed n times, followed by a pause of 1 second, the system willtreat that as the end of
sequence for that button and decode the entered character. If a key is pressed n times, and then a different key is entered,this will indicate an end of the previous entry and start of a new entry. The machine registers the output of the previous entry.

![Keypad Used](https://components101.com/sites/default/files/components/4x4-Keypad-Module.jpg)

## Implementation
The FSM was implemented in C. The system was tested with a Tiva TM4C123 Controller.