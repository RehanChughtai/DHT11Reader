dhtPin = 2
buttonPin = 7
gpio.mode(buttonPin, gpio.INPUT)
gpio.write(buttonPin, gpio.LOW)
pushed = 0
mytimer = tmr.create()

if not mytimer:alarm(5000, tmr.ALARM_AUTO, function()
status, temp, humi, temp_dec, humi_dec = dht.read11(dhtPin)

register(100, 1, function() 
if gpio.read(buttonPin)==1 and pushed == 0 then 
    pushed = 1
    print("Button detected")
    
if status == dht.OK then
    print("DHT Temperature:"..temp..";".."Humidity:"..humi)
-- 2 dots are used for concatenation
elseif status == dht.ERROR_CHECKSUM then
    print( "DHT Checksum error." )
elseif status == dht.ERROR_TIMEOUT then
    print( "DHT timed out." )
end
  print("Temperature and Humidity Given")
end)
then
  print("Whoops")
end

end
end)

--This one can only allow you to detect push for once and only once

mytimer:start()