# MIDAS-Software
Illinois Space Society's flight software codebase for the MIDAS avionics system

### Directory Structure:
- `eMMC/`: Code used to extract a MIDAS log file from the onaboard eMMC.
- `ground/`: Code running on ground station hardware (Adafruit LoRa Feather + combiner laptop).
- `MIDAS/`: Mission critical flight software running on MIDAS. This is the code that actually flies on the rocket.
    - `docs/`: Files related to documentation.
	- `lib/`: Hardware drivers to interface with sensors and more. Some are downloaded from the internet and some were written by us.- `src/`: All the flight code that we write ourselves is in this directory. 

### Avionics Software Team 23-24:
Aidan Costello,
Magilan Sendhil,
Aaditya Voruganti,
Michael Karpov,
Zyun Lam,
Surag Nuthulapaty,
Nicholas Phillips,
Gautam Dayal,
Rithvik Bhogavilli,
Anthony Smykalov,
Aryaman Dwivedi,
Eisha Peyyeti,
Rishi Gottumukkala,
Carson Sprague,
Deeya Bodas,
Ashley Sawa,
Ankith Madadi,
Patrick Cassino,
Kaustubh Khulbe,
Ethan Pereira