import time

adcSensitivity = 0.00489
maxCurrent = float(input("max current? (A) "))
resistance = float(input("resistance? (R) "))
#adcSteps   = int(input("desired minimal ADC steps?: "))

minCurrent = adcSensitivity / resistance
dissipation = maxCurrent * maxCurrent * resistance

print()
print("ADC sensitivity = {}V/step".format(adcSensitivity))
print("Min current     = {}mA/step".format(1000*minCurrent))
print("Max dissipation = {}W".format(dissipation ))
print()

print("finding suitable package...")

packages =     ["0201","0402", "0603", "0805", "1206", "1210", "2010", "2512" ]
dissipations = [ 0.05,  0.0625, 0.0625,    0.1, 0.125,   0.25,   0.25, 0.50, ]

#print( "resistance: R{}, dissipation: {}W".format(resistance, powah ))
print("searching for {}W".format(dissipation))
for i in range(0, 7):
    #print( "package {} -> {}W".format(packages[i],dissipations[i]))
    if dissipation < dissipations[i]:
        print("RECOMMENDED PACKAGE = {}!!".format(packages[i] ))
        exit()
else:
    print( "\r\n!!!TO MUCH POWER DISSIPATION!!!" )
    print( "!!!2 RESISTORS IN PARALLEL ARE NEEDED!!!\r\n")

dissipation /= 4
print("searching for {}W".format(dissipation))
for i in range(0, 7):
    #print( "package {} -> {}W".format(packages[i],dissipations[i]))
    if dissipation < dissipations[i]:
        print("RECOMMENDED PACKAGE = {}!!".format(packages[i] ))
        exit()
else:
    print( "STILL TO MUCH POWER DISSIPATION!!" )
    print( "Pick lower shunt or lower maximum current" )

