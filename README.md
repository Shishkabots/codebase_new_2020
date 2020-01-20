Notes:
  How the "commands" structure works:
    1. Do what is written in the "initialize" first
    2. Run the "execute" method and then check the "isFinished" to see if the condition is true and the code is finished
      3. if it is finished, then go to "end" and run whatever is in there
      4. else if it is not finished, then repeat "execute" and then check "isfinished" again
   
   Last year's code:
    1. robot.java calls telopcommands.java which is a filler command
    2. teleopcommands.java has all of the "addSequential"/ "addParallel" functions that will call the actual commands such as turning the hatch or shooting the cargo(LAST YR's CHALLENGE)
    3. You need the teleopcommands.java in the middle due to the way frc command based programming works (i.e. you cant just do "addSequential/Parallel" from robot.java)
