Thingsboard Authentication
1. Access Token based Authentication
Access token: d54r7ecmqs40hcc6o58h
- Plain MQTT (without SSL)
mosquitto_pub -d -q 1 -h "localhost" -p "1883" -t "v1/devices/me/telemetry" -u "d54r7ecmqs40hcc6o58h" -m {"temperature":25}

- MQTTS (MQTT over SSL)
One-way SSL authentication is a standard authentication mode, where your client device verifies 
the identity of a server using server certificate. 

Configure the following environment variables via configuration file, docker-compose
Self-signed certificates generation
Use instructions below to generate your own certificate files. Useful for tests, but time consuming and not recommended for production.

PEM certificate file
Note This step requires Linux based OS with openssl installed.
To generate a server self-signed PEM certificate and private key, use the following command:
openssl ecparam -out server_key.pem -name secp256r1 -genkey
openssl req -new -key server_key.pem -x509 -nodes -days 365 -out server.pem 

mosquitto_pub --cafile server.pem -d -q 1 -h "192.168.1.40" -p "8883" -t "v1/devices/me/telemetry" -u "v8pr0qvr1raxi3w0jxt0" -m {"temperature":25}

2. MQTT Basic Credentials
{clientId:"s210baicgmd8w4hthfjz",userName:"sxcl1ah73lph94lt3i5b",password:"k9ko7yggkl4qkqc7ihws"}
mosquitto_pub -d -q 1 -h localhost -p 1883 -t v1/devices/me/telemetry -i s210baicgmd8w4hthfjz -u sxcl1ah73lph94lt3i5b -P k9ko7yggkl4qkqc7ihws -m "{temperature:25}"

3. X509 Certificate based Authentication
Step 1. Prepare your server and certificate chain
Follow the MQTT over SSL guide to provision server certificate if you are hosting your own ThingsBoard instance.

Step 2. Generate Client certificate
Use the following command to generate the self-signed private key and x509 certificate.

To generate the EC based key and certificate, use:
openssl ecparam -out key.pem -name secp256r1 -genkey
openssl req -new -key key.pem -x509 -nodes -days 365 -out cert.pem 

mosquitto_pub --cafile server.pem -d -q 1 -h "192.168.1.40" -p "8883" \
-t "v1/devices/me/telemetry" --key key.pem --cert cert.pem -m {"temperature":25}

mosquitto_pub options
-d,
  --debug
    Enable debug messages.
-i,
  --id
    The id to use for this client. 
-P,
  --pw
    Provide a password to be used for authenticating with the broker. 
	Using this argument without also specifying a username is invalid when using MQTT v3.1 or v3.1.1. 

-u,
  --username
    Provide a username to be used for authenticating with the broker. See also the --pw argument.
	
--cafile
  Define the path to a file containing PEM encoded CA certificates that are trusted. Used to enable SSL communication.

--capath
  Define the path to a directory containing PEM encoded CA certificates that are trusted. Used to enable SSL communication.
  For --capath to work correctly, the certificate files must have ".crt" as the file ending and you must run "openssl rehash <path to capath>" each time you add/remove a certificate.

--cert
  Define the path to a file containing a PEM encoded certificate for this client, if required by the server.
--key
  Define the path to a file containing a PEM encoded private key for this client, if required by the server.

