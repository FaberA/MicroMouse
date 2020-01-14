# MicroMouse

Micro Mouse Code - Competition 2020

MEMO Number:  MM-XII- Software Requirements Specifications (SRS)

DATE: 01-JAN-2019

TO:  Dr. EFC LaBerge

FROM:  Zachary Kempler, Richard Ensor, Shahbaz Khan, Faber Alarcon

SUBJECT:  MM-XII Software Requirements Specifications (SRS)

MM-XII Software Requirements Specification (SRS)

1.	Introduction

  1.1.	PROJECT DESCRIPTION

  1.2.	EXTERNAL STANDARDS

    •	Interface used to program and debug the microcontroller.

    •	Interface used to connect the onboard power source to components.

    •	Interface to replenish onboard power source prior to use.

  1.3.	REFERENCED DOCUMENTS

    •	The IEEE Micromouse 2019 competition rules (https://site.ieee.org/r1/files/2019/01/2019-R1-Micromouse-Guideline.pdf)

  1.4.	OVERVIEW OF DOCUMENT

    The requirements laid out in this document depend heavily on the IEEE competition rules and requirements. Functional, design, and performance-based requirements are all designed in order to optimize speed and reduce overall run-time while solving the maze.

2.	 System Requirements

  2.1.		FUNCTIONAL REQUIREMENTS

    2.1.1.	CREATE DATA STRUCTURE TO STORE THE MAZE

      2.1.1.1.	The maze shall be populated using a data time and space efficient data structure in reference to worst case BIG O complexity.  

      2.1.1.2.	The maze walls in the maze shall be empty i.e no node shall have a connection upon initialization of a new scanning run

      2.1.1.3.	The maze walls shall be populated as the bot traverses the maze using information obtained via the infrared sensors. 

      2.1.1.4.	The scanning run is completed once each box has been visited in the maze 

      2.1.1.5.	The maze shall be output to a human readable text file upon completion of the scanning run.

      2.1.1.6.	The output text file functionality shall be optional and only occur when the bot is set to a testing mode. 

    2.1.2.	 IMPLEMENT THE FLOOD FILL ALGORITHM ON THE STORED REPRESENTATION OF THE MAZE

      2.1.2.1.	The MM shall

    2.1.3.	DETERMINE WEIGHTED VALUE OF OPTIMAL PATHS BASED ON PHYSICAL CONSTRAINTS

      2.1.3.1.	The MM shall 

  2.2.	PERFORMANCE REQUIREMENTS

     2.2.1.	TIMING

      There are 10 minutes allotted for the MM to solve and complete the maze. The MM shall give enough time to scan the maze, solve the optimal path for the maze, and execute the optimal path from the beginning of the maze. The time that the scanning and solving states take shall leave at least 1 minute for the optimal path to be executed. Therefore, the maximum time allowed for the MM to scan the environment and solve for the optimal path shall be no greater than 9 minutes.

    2.2.2.	ACCURACY

      The walls shall be populated in the acurate positions. 

  2.3.	INTERFACE REQUIREMENTS

    2.3.1.	The interface for the MM shall allow for generation and download of a human readable representation of the maze during testing mode of operation. 

    2.3.2.	The hardware components shall be capable of storing the internal representation of the maze.  

  2.4.	TESTING REQUIREMENTS

     2.4.1.	The MM shall have a testing mode of operation such that the internal representation of the scanned maze can be analyzed in human readable format. 

     2.4.2.	The MM shall generate additional representations for solved paths and weighted paths based on physical constraints as well as a final optimal path 
