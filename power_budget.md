500kJ battery (12V 12Ah)
350kJ Assume 70% efficiency (battery wear + power loss).

Minimum operation time: 2h
Maximum power consumption (when all is full power): 50W


Power Budget: with Li-ion battery: 12V 100Ah -> 4.32MJ
~4.5h of operation @ 265W

Power Consumption:					Idle		Full
Orange pi 5+						1.8W		8.3W
Intel Realsense Camera D435i		-			3.675W
Pneumatic air compressor			0W			240W

Air flow rate (with 2 compressors @ 200 PSI): 3.6 CFM
piston * (4/leg * 6leg) = 24 pistons
24 pistons * 2 discharge/sec * ? CFP = ? CFM


Weight Budget: 25,000g ?
RPi 3B+
Intel Realsense Camera D435i		    75 g
2 Li-ion batteries parallel			50,000 g


Motor Torque calculation:
    τ_1_max = l_1 * m/2 * g
    τ_2_max = (l_1 + l_2) * m/2 * g
