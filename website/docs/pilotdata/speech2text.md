---
sidebar_position: 5
id: speech2text
title: Speech2Text Output Data
---

# Speech2Text Output Data

Speech2Text Output Data provides text strings generated by ASR (Automatic Speech Recognition) with timestamps and confidence rating.

Each recording has two .csv files that are the same, except  `speech2text/speech.csv` uses the wav file time domain and  `speech2text/speech_aria_domain.csv` uses Aria time domain.



**Table 1:** *`speech.csv` Structure*

|startTime_ms          |endTime_ms          |written          |confidence          |
|---          |---          |---          |---          |
|54040          |55040          | I’m          |0.25608          |
|72920          |73920          | looking          |0.84339          |

*Note:* token in wav file time domain (start = 0)



**Table 2:** *`speech_aria_domain.csv` Structure*

| startTime_ns          | endTime_ns          | written          | confidence          |
|---          |---          |---          |---          |
|56511040          |56512040          | I’m          |0.25608          |
|56529920          |56530920          | looking          |0.84339          |

*Note:* token in Aria file time domain (start = 0)