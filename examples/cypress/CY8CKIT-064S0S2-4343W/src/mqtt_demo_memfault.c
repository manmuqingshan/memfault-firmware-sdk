/*
 * FreeRTOS V202012.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 */

/*
 * Demo for showing use of the MQTT API using a mutually authenticated
 * network connection.
 *
 * The Example shown below uses MQTT APIs to create MQTT messages and send them
 * over the mutually authenticated network connection established with the
 * MQTT broker. This example is single threaded and uses statically allocated
 * memory. It uses QoS1 for sending to and receiving messages from the broker.
 *
 * A mutually authenticated TLS connection is used to connect to the
 * MQTT message broker in this example. Define democonfigMQTT_BROKER_ENDPOINT
 * and democonfigROOT_CA_PEM, in mqtt_demo_mutual_auth_config.h, and the client
 * private key and certificate, in aws_clientcredential_keys.h, to establish a
 * mutually authenticated connection.
 *
 * This example is based on the
 * <amazon-freertos>/demos/coreMQTT/mqtt_demo_mutual_auth.c , modified to
 * publish any available memfault chunks ('memfault_packetizer_get_chunk') to
 * the MQTT 'mqttexampleTOPIC'.
 */

/**
 * @file mqtt_demo_memfault.c
 * @brief Demonstrates usage of the MQTT library.
 */

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Demo Specific configs. */
#include "mqtt_demo_mutual_auth_config.h"

/* Include common demo header. */
#include "aws_demo.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* MQTT library includes. */
#include "core_mqtt.h"

/* Retry utilities include. */
#include "backoff_algorithm.h"

/* Include PKCS11 helpers header. */
#include "pkcs11_helpers.h"

/* Transport interface implementation include header for TLS. */
#include "transport_secure_sockets.h"

/* Include header for connection configurations. */
#include "aws_clientcredential.h"

/* Include header for client credentials. */
#include "aws_clientcredential_keys.h"

/* Include header for root CA certificates. */
#include "iot_default_root_certificates.h"

/* Include AWS IoT metrics macros header. */
#include "aws_iot_metrics.h"

/* Include memfault headers */
#include <memfault/components.h>
#include <memfault/ports/freertos.h>
#include <memfault_test.h>

/*------------- Demo configurations -------------------------*/

/** Note: The device client certificate and private key credentials are
 * obtained by the transport interface implementation (with Secure Sockets)
 * from the demos/include/aws_clientcredential_keys.h file.
 *
 * The following macros SHOULD be defined for this demo which uses both server
 * and client authentications for TLS session:
 *   - keyCLIENT_CERTIFICATE_PEM for client certificate.
 *   - keyCLIENT_PRIVATE_KEY_PEM for client private key.
 */

/**
 * @brief The MQTT broker endpoint used for this demo.
 */
#ifndef democonfigMQTT_BROKER_ENDPOINT
  #define democonfigMQTT_BROKER_ENDPOINT clientcredentialMQTT_BROKER_ENDPOINT
#endif

/**
 * @brief The root CA certificate belonging to the broker.
 */
#ifndef democonfigROOT_CA_PEM
  #define democonfigROOT_CA_PEM tlsATS1_ROOT_CERTIFICATE_PEM
#endif

#ifndef democonfigCLIENT_IDENTIFIER

/**
 * @brief The MQTT client identifier used in this example.  Each client identifier
 * must be unique so edit as required to ensure no two clients connecting to the
 * same broker use the same client identifier.
 */
  #define democonfigCLIENT_IDENTIFIER clientcredentialIOT_THING_NAME
#endif

#ifndef democonfigMQTT_BROKER_PORT

/**
 * @brief The port to use for the demo.
 */
  #define democonfigMQTT_BROKER_PORT clientcredentialMQTT_BROKER_PORT
#endif

/**
 * @brief The maximum number of times to run the subscribe publish loop in this
 * demo.
 */
#ifndef democonfigMQTT_MAX_DEMO_COUNT
  #define democonfigMQTT_MAX_DEMO_COUNT (3)
#endif
/*-----------------------------------------------------------*/

/**
 * @brief The maximum number of retries for network operation with server.
 */
#define RETRY_MAX_ATTEMPTS (5U)

/**
 * @brief The maximum back-off delay (in milliseconds) for retrying failed operation
 *  with server.
 */
#define RETRY_MAX_BACKOFF_DELAY_MS (5000U)

/**
 * @brief The base back-off delay (in milliseconds) to use for network operation retry
 * attempts.
 */
#define RETRY_BACKOFF_BASE_MS (500U)

/**
 * @brief Timeout for receiving CONNACK packet in milliseconds.
 */
#define mqttexampleCONNACK_RECV_TIMEOUT_MS (1000U)

/**
 * @brief The topic to subscribe and publish to in the example.
 *
 * Contains fingerprint and project key for forwarding to memfault:
 *   prod/<board>/<device_id>/memfault/<memfault_project_key>/chunk
 */
#define mqttexampleTOPIC                                 \
  "prod/psoc64/" democonfigCLIENT_IDENTIFIER "/memfault" \
  "/" clientcredentialMEMFAULT_PROJECT_KEY "/chunk"
#define mqttforwardTOPIC "device/" democonfigCLIENT_IDENTIFIER "/forward"

/**
 * @brief The number of topic filters to subscribe.
 */
#define mqttexampleTOPIC_COUNT (1)

/**
 * @brief The MQTT message published in this example.
 */
#define mqttexampleMESSAGE "Hello World!"

/**
 * @brief Time in ticks to wait between each cycle of the demo implemented
 * by RunCoreMqttMemfaultDemo().
 */
#define mqttexampleDELAY_BETWEEN_DEMO_ITERATIONS_TICKS (pdMS_TO_TICKS(5000U))

/**
 * @brief Timeout for MQTT_ProcessLoop in milliseconds.
 */
#define mqttexamplePROCESS_LOOP_TIMEOUT_MS (700U)

/**
 * @brief The maximum number of times to call MQTT_ProcessLoop() when polling
 * for a specific packet from the broker.
 */
#define MQTT_PROCESS_LOOP_PACKET_WAIT_COUNT_MAX (30U)

/**
 * @brief Keep alive time reported to the broker while establishing
 * an MQTT connection.
 *
 * It is the responsibility of the Client to ensure that the interval between
 * Control Packets being sent does not exceed the this Keep Alive value. In the
 * absence of sending any other Control Packets, the Client MUST send a
 * PINGREQ Packet.
 */
#define mqttexampleKEEP_ALIVE_TIMEOUT_SECONDS (60U)

/**
 * @brief Delay (in ticks) between consecutive cycles of MQTT publish operations in a
 * demo iteration.
 *
 * Note that the process loop also has a timeout, so the total time between
 * publishes is the sum of the two delays.
 */
#define mqttexampleDELAY_BETWEEN_PUBLISHES_TICKS (pdMS_TO_TICKS(5000U))

/**
 * @brief Transport timeout in milliseconds for transport send and receive.
 */
#define mqttexampleTRANSPORT_SEND_RECV_TIMEOUT_MS (500U)

/**
 * @brief Milliseconds per second.
 */
#define MILLISECONDS_PER_SECOND (1000U)

/**
 * @brief Milliseconds per FreeRTOS tick.
 */
#define MILLISECONDS_PER_TICK (MILLISECONDS_PER_SECOND / configTICK_RATE_HZ)

/**
 * @brief Memfault chunk size.
 */
#define MEMFAULT_CHUNK_SIZE (1500U)

/**
 * @brief Chunk upload delay.
 */
#define mqttexampleDELAY_UPLOAD (pdMS_TO_TICKS(5000U))

/*-----------------------------------------------------------*/

/**
 * @brief Each compilation unit that consumes the NetworkContext must define it.
 * It should contain a single pointer to the type of your desired transport.
 * When using multiple transports in the same compilation unit, define this pointer as void *.
 *
 * @note Transport stacks are defined in
 * amazon-freertos/libraries/abstractions/transport/secure_sockets/transport_secure_sockets.h.
 */
struct NetworkContext {
  SecureSocketsTransportParams_t *pParams;
};

/*-----------------------------------------------------------*/

/**
 * @brief Calculate and perform an exponential backoff with jitter delay for
 * the next retry attempt of a failed network operation with the server.
 *
 * The function generates a random number, calculates the next backoff period
 * with the generated random number, and performs the backoff delay operation if the
 * number of retries have not exhausted.
 *
 * @note The PKCS11 module is used to generate the random number as it allows access
 * to a True Random Number Generator (TRNG) if the vendor platform supports it.
 * It is recommended to seed the random number generator with a device-specific entropy
 * source so that probability of collisions from devices in connection retries is mitigated.
 *
 * @note The backoff period is calculated using the backoffAlgorithm library.
 *
 * @param[in, out] pxRetryAttempts The context to use for backoff period calculation
 * with the backoffAlgorithm library.
 *
 * @return pdPASS if calculating the backoff period was successful; otherwise pdFAIL
 * if there was failure in random number generation OR all retry attempts had exhausted.
 */
static BaseType_t prvBackoffForRetry(BackoffAlgorithmContext_t *pxRetryParams);

/**
 * @brief Connect to MQTT broker with reconnection retries.
 *
 * If connection fails, retry is attempted after a timeout.
 * Timeout value will exponentially increase until maximum
 * timeout value is reached or the number of attempts are exhausted.
 *
 * @param[out] pxNetworkContext The output parameter to return the created network context.
 *
 * @return pdFAIL on failure; pdPASS on successful TLS+TCP network connection.
 */
static BaseType_t prvConnectToServerWithBackoffRetries(NetworkContext_t *pNetworkContext);

/**
 * @brief Sends an MQTT Connect packet over the already connected TLS over TCP connection.
 *
 * @param[in, out] pxMQTTContext MQTT context pointer.
 * @param[in] xNetworkContext Network context.
 *
 * @return pdFAIL on failure; pdPASS on successful MQTT connection.
 */
static BaseType_t prvCreateMQTTConnectionWithBroker(MQTTContext_t *pxMQTTContext,
                                                    NetworkContext_t *pxNetworkContext);

/**
 * @brief Function to update variable #xTopicFilterContext with status
 * information from Subscribe ACK. Called by the event callback after processing
 * an incoming SUBACK packet.
 *
 * @param[in] pxPacketInfo Server response to the subscription request.
 */
static void prvUpdateSubAckStatus(MQTTPacketInfo_t *pxPacketInfo);

/**
 * @brief Publishes a message mqttexampleMESSAGE on mqttexampleTOPIC topic.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 *
 * @return pdFAIL on failure; pdPASS on successful PUBLISH operation.
 */
static BaseType_t prvMQTTPublishToTopic(MQTTContext_t *pxMQTTContext, const void *payload,
                                        size_t len);

/**
 * @brief The timer query function provided to the MQTT context.
 *
 * @return Time in milliseconds.
 */
static uint32_t prvGetTimeMs(void);

/**
 * @brief Process a response or ack to an MQTT request (PING, PUBLISH,
 * SUBSCRIBE or UNSUBSCRIBE). This function processes PINGRESP, PUBACK,
 * SUBACK, and UNSUBACK.
 *
 * @param[in] pxIncomingPacket is a pointer to structure containing deserialized
 * MQTT response.
 * @param[in] usPacketId is the packet identifier from the ack received.
 */
static void prvMQTTProcessResponse(MQTTPacketInfo_t *pxIncomingPacket, uint16_t usPacketId);

/**
 * @brief Process incoming Publish message.
 *
 * @param[in] pxPublishInfo is a pointer to structure containing deserialized
 * Publish message.
 */
static void prvMQTTProcessIncomingPublish(MQTTPublishInfo_t *pxPublishInfo);

/**
 * @brief The application callback function for getting the incoming publishes,
 * incoming acks, and ping responses reported from the MQTT library.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 * @param[in] pxPacketInfo Packet Info pointer for the incoming packet.
 * @param[in] pxDeserializedInfo Deserialized information from the incoming packet.
 */
static void prvEventCallback(MQTTContext_t *pxMQTTContext, MQTTPacketInfo_t *pxPacketInfo,
                             MQTTDeserializedInfo_t *pxDeserializedInfo);

/*-----------------------------------------------------------*/

/**
 * @brief Static buffer used to hold MQTT messages being sent and received.
 */
static uint8_t ucSharedBuffer[democonfigNETWORK_BUFFER_SIZE];

/**
 * @brief Global entry time into the application to use as a reference timestamp
 * in the #prvGetTimeMs function. #prvGetTimeMs will always return the difference
 * between the current time and the global entry time. This will reduce the chances
 * of overflow for the 32 bit unsigned integer used for holding the timestamp.
 */
static uint32_t ulGlobalEntryTimeMs;

/**
 * @brief Packet Identifier generated when Publish request was sent to the broker;
 * it is used to match received Publish ACK to the transmitted Publish packet.
 */
static uint16_t usPublishPacketIdentifier;

/**
 * @brief Packet Identifier generated when Subscribe request was sent to the broker;
 * it is used to match received Subscribe ACK to the transmitted Subscribe packet.
 */
static uint16_t usSubscribePacketIdentifier;

/**
 * @brief Packet Identifier generated when Unsubscribe request was sent to the broker;
 * it is used to match received Unsubscribe response to the transmitted Unsubscribe
 * request.
 */
static uint16_t usUnsubscribePacketIdentifier;

/**
 * @brief MQTT packet type received from the MQTT broker.
 *
 * @note Only on receiving incoming PUBLISH, SUBACK, and UNSUBACK, this
 * variable is updated. For MQTT packets PUBACK and PINGRESP, the variable is
 * not updated since there is no need to specifically wait for it in this demo.
 * A single variable suffices as this demo uses single task and requests one operation
 * (of PUBLISH, SUBSCRIBE, UNSUBSCRIBE) at a time before expecting response from
 * the broker. Hence it is not possible to receive multiple packets of type PUBLISH,
 * SUBACK, and UNSUBACK in a single call of #prvWaitForPacket.
 * For a multi task application, consider a different method to wait for the packet, if needed.
 */
static uint16_t usPacketTypeReceived = 0U;

/**
 * @brief A pair containing a topic filter and its SUBACK status.
 */
typedef struct topicFilterContext {
  const char *pcTopicFilter;
  MQTTSubAckStatus_t xSubAckStatus;
} topicFilterContext_t;

/**
 * @brief An array containing the context of a SUBACK; the SUBACK status
 * of a filter is updated when the event callback processes a SUBACK.
 */
static topicFilterContext_t xTopicFilterContext[mqttexampleTOPIC_COUNT] = { { mqttforwardTOPIC,
                                                                              MQTTSubAckFailure } };

/** @brief Static buffer used to hold MQTT messages being sent and received. */
static MQTTFixedBuffer_t xBuffer = { ucSharedBuffer, democonfigNETWORK_BUFFER_SIZE };

/*-----------------------------------------------------------*/

/*
 * @brief The example shown below uses MQTT APIs to create MQTT messages and
 * send them over the mutually authenticated network connection established with the
 * MQTT broker. This example is single threaded and uses statically allocated
 * memory. It uses QoS1 for sending to and receiving messages from the broker.
 *
 * This MQTT client subscribes to the topic as specified in mqttexampleTOPIC at the
 * top of this file by sending a subscribe packet and then waiting for a subscribe
 * acknowledgment (SUBACK).This client will then publish to the same topic it
 * subscribed to, so it will expect all the messages it sends to the broker to be
 * sent back to it from the broker.
 *
 * This example runs for democonfigMQTT_MAX_DEMO_COUNT, if the
 * connection to the broker goes down, the code tries to reconnect to the broker
 * with an exponential backoff mechanism.
 */
int RunCoreMqttMemfaultDemo(bool awsIotMqttMode, const char *pIdentifier, void *pNetworkServerInfo,
                            void *pNetworkCredentialInfo,
                            const IotNetworkInterface_t *pNetworkInterface) {
  uint32_t ulDemoRunCount = 0U;
  NetworkContext_t xNetworkContext = { 0 };
  MQTTContext_t xMQTTContext = { 0 };
  TransportSocketStatus_t xNetworkStatus;
  BaseType_t xIsConnectionEstablished = pdFALSE;
  SecureSocketsTransportParams_t secureSocketsTransportParams = { 0 };
  uint8_t mfltBuf[MEMFAULT_CHUNK_SIZE];
  size_t mfltLen = sizeof(mfltBuf);
  bool isDataAvailable = false;
  bool isDemoSuccessful = false;

  /* Upon return, pdPASS will indicate a successful demo execution.
   * pdFAIL will indicate some failures occurred during execution. The
   * user of this demo must check the logs for any failure codes. */
  BaseType_t xDemoStatus = pdFAIL;

  /* Remove compiler warnings about unused parameters. */
  (void)awsIotMqttMode;
  (void)pIdentifier;
  (void)pNetworkServerInfo;
  (void)pNetworkCredentialInfo;
  (void)pNetworkInterface;

  /* Set the entry time of the demo application. This entry time will be used
   * to calculate relative time elapsed in the execution of the demo application,
   * by the timer utility function that is provided to the MQTT library.
   */
  ulGlobalEntryTimeMs = prvGetTimeMs();
  xNetworkContext.pParams = &secureSocketsTransportParams;

  for (;;) {
    /* Attempt to establish TLS session with MQTT broker. If connection fails,
     * retry after a timeout. Timeout value will be exponentially increased until
     * the maximum number of attempts are reached or the maximum timeout value is reached.
     * The function returns a failure status if the TLS over TCP connection cannot be established
     * to the broker after the configured number of attempts. */
    xDemoStatus = prvConnectToServerWithBackoffRetries(&xNetworkContext);

    if (xDemoStatus == pdPASS) {
      /* Set a flag indicating a TLS connection exists. This is done to
       * disconnect if the loop exits before disconnection happens. */
      xIsConnectionEstablished = pdTRUE;

      /* Sends an MQTT Connect packet over the already established TLS connection,
       * and waits for connection acknowledgment (CONNACK) packet. */
      LogInfo(("Creating an MQTT connection to %s.", democonfigMQTT_BROKER_ENDPOINT));
      xDemoStatus = prvCreateMQTTConnectionWithBroker(&xMQTTContext, &xNetworkContext);
    }

    if (xDemoStatus == pdPASS) {
      LogInfo(("Retrieving data chunk."));
      isDataAvailable = memfault_packetizer_get_chunk(mfltBuf, &mfltLen);

      while (isDataAvailable) {
        LogInfo(("Publishing to %s.", mqttexampleTOPIC));
        xDemoStatus = prvMQTTPublishToTopic(&xMQTTContext, mfltBuf, mfltLen);

        vTaskDelay(mqttexampleDELAY_UPLOAD);

        if (xDemoStatus == pdPASS) {
          LogInfo(("Publish success."));
          mfltLen = sizeof(mfltBuf);
          isDataAvailable = memfault_packetizer_get_chunk(mfltBuf, &mfltLen);
        } else {
          LogInfo(("Publish failure."));
          isDataAvailable = false;
        }
      }
    }

    if (xDemoStatus == pdPASS) {
      /* Send an MQTT Disconnect packet over the already connected TLS over TCP connection.
       * There is no corresponding response for the disconnect packet. After sending
       * disconnect, client must close the network connection. */
      LogInfo(("Disconnecting the MQTT connection with %s.", democonfigMQTT_BROKER_ENDPOINT));
      MQTT_Disconnect(&xMQTTContext);
    }

    /* We will always close the network connection, even if an error may have occurred during
     * demo execution, to clean up the system resources that it may have consumed. */
    if (xIsConnectionEstablished == pdTRUE) {
      /* Close the network connection. */
      xNetworkStatus = SecureSocketsTransport_Disconnect(&xNetworkContext);

      if (xNetworkStatus != TRANSPORT_SOCKET_STATUS_SUCCESS) {
        xDemoStatus = pdFAIL;
        LogError(("SecureSocketsTransport_Disconnect() failed to close the network connection. "
                  "StatusCode=%d.",
                  (int)xNetworkStatus));
      }
    }

    if (xDemoStatus == pdPASS) {
      isDemoSuccessful = true;
      LogInfo(("Demo completed successfully."));
      break;
    } else {
      /* Demo loop will be repeated for up to democonfigMQTT_MAX_DEMO_COUNT
       * times if current loop resulted in a failure. */
      isDemoSuccessful = false;
      if (ulDemoRunCount >= democonfigMQTT_MAX_DEMO_COUNT) {
        LogInfo(("Demo failed, exceeded restart limit."));
        break;
      } else {
        LogInfo(("Demo failed, restarting after a delay."));
        ulDemoRunCount++;
        vTaskDelay(mqttexampleDELAY_BETWEEN_DEMO_ITERATIONS_TICKS);
      }
    }
  }

  if (isDemoSuccessful) {
    xDemoStatus = pdPASS;
    LogInfo(("Demo run successful."));
    for (;;);
  } else {
    xDemoStatus = pdFAIL;
  }

  return (xDemoStatus == pdPASS) ? EXIT_SUCCESS : EXIT_FAILURE;
}
/*-----------------------------------------------------------*/

static BaseType_t prvBackoffForRetry(BackoffAlgorithmContext_t *pxRetryParams) {
  BaseType_t xReturnStatus = pdFAIL;
  uint16_t usNextRetryBackOff = 0U;
  BackoffAlgorithmStatus_t xBackoffAlgStatus = BackoffAlgorithmSuccess;

  /**
   * To calculate the backoff period for the next retry attempt, we will
   * generate a random number to provide to the backoffAlgorithm library.
   *
   * Note: The PKCS11 module is used to generate the random number as it allows access
   * to a True Random Number Generator (TRNG) if the vendor platform supports it.
   * It is recommended to use a random number generator seeded with a device-specific
   * entropy source so that probability of collisions from devices in connection retries
   * is mitigated.
   */
  uint32_t ulRandomNum = 0;

  if (xPkcs11GenerateRandomNumber((uint8_t *)&ulRandomNum, sizeof(ulRandomNum)) == pdPASS) {
    /* Get back-off value (in milliseconds) for the next retry attempt. */
    xBackoffAlgStatus =
      BackoffAlgorithm_GetNextBackoff(pxRetryParams, ulRandomNum, &usNextRetryBackOff);

    if (xBackoffAlgStatus == BackoffAlgorithmRetriesExhausted) {
      LogError(("All retry attempts have exhausted. Operation will not be retried"));
    } else if (xBackoffAlgStatus == BackoffAlgorithmSuccess) {
      /* Perform the backoff delay. */
      vTaskDelay(pdMS_TO_TICKS(usNextRetryBackOff));

      xReturnStatus = pdPASS;

      LogInfo(("Retry attempt %lu out of maximum retry attempts %lu.",
               (pxRetryParams->attemptsDone + 1), pxRetryParams->maxRetryAttempts));
    }
  } else {
    LogError(("Unable to retry operation with broker: Random number generation failed"));
  }

  return xReturnStatus;
}

/*-----------------------------------------------------------*/

static BaseType_t prvConnectToServerWithBackoffRetries(NetworkContext_t *pxNetworkContext) {
  ServerInfo_t xServerInfo = { 0 };

  SocketsConfig_t xSocketsConfig = { 0 };
  TransportSocketStatus_t xNetworkStatus = TRANSPORT_SOCKET_STATUS_SUCCESS;
  BackoffAlgorithmContext_t xReconnectParams;
  BaseType_t xBackoffStatus = pdFALSE;

  /* Set the credentials for establishing a TLS connection. */
  /* Initializer server information. */
  xServerInfo.pHostName = democonfigMQTT_BROKER_ENDPOINT;
  xServerInfo.hostNameLength = strlen(democonfigMQTT_BROKER_ENDPOINT);
  xServerInfo.port = democonfigMQTT_BROKER_PORT;

  /* Configure credentials for TLS mutual authenticated session. */
  xSocketsConfig.enableTls = true;
  xSocketsConfig.pAlpnProtos = NULL;
  xSocketsConfig.maxFragmentLength = 0;
  xSocketsConfig.disableSni = false;
  xSocketsConfig.pRootCa = democonfigROOT_CA_PEM;
  xSocketsConfig.rootCaSize = sizeof(democonfigROOT_CA_PEM);
  xSocketsConfig.sendTimeoutMs = mqttexampleTRANSPORT_SEND_RECV_TIMEOUT_MS;
  xSocketsConfig.recvTimeoutMs = mqttexampleTRANSPORT_SEND_RECV_TIMEOUT_MS;

  /* Initialize reconnect attempts and interval. */
  BackoffAlgorithm_InitializeParams(&xReconnectParams, RETRY_BACKOFF_BASE_MS,
                                    RETRY_MAX_BACKOFF_DELAY_MS, RETRY_MAX_ATTEMPTS);

  /* Attempt to connect to MQTT broker. If connection fails, retry after
   * a timeout. Timeout value will exponentially increase till maximum
   * attempts are reached.
   */
  do {
    /* Establish a TLS session with the MQTT broker. This example connects to
     * the MQTT broker as specified in democonfigMQTT_BROKER_ENDPOINT and
     * democonfigMQTT_BROKER_PORT at the top of this file. */
    LogInfo(("Creating a TLS connection to %s:%u.", democonfigMQTT_BROKER_ENDPOINT,
             democonfigMQTT_BROKER_PORT));
    /* Attempt to create a mutually authenticated TLS connection. */
    xNetworkStatus =
      SecureSocketsTransport_Connect(pxNetworkContext, &xServerInfo, &xSocketsConfig);

    if (xNetworkStatus != TRANSPORT_SOCKET_STATUS_SUCCESS) {
      LogWarn(
        ("Connection to the broker failed. Attempting connection retry after backoff delay."));

      /* As the connection attempt failed, we will retry the connection after an
       * exponential backoff with jitter delay. */

      /* Calculate the backoff period for the next retry attempt and perform the wait operation. */
      xBackoffStatus = prvBackoffForRetry(&xReconnectParams);
    } else {
      LogInfo(("Connection attempt success."));
    }
  } while ((xNetworkStatus != TRANSPORT_SOCKET_STATUS_SUCCESS) && (xBackoffStatus == pdPASS));

  LogInfo(("Connection attempt finished."));

  return (xNetworkStatus == TRANSPORT_SOCKET_STATUS_SUCCESS) ? pdPASS : pdFAIL;
}
/*-----------------------------------------------------------*/

static BaseType_t prvCreateMQTTConnectionWithBroker(MQTTContext_t *pxMQTTContext,
                                                    NetworkContext_t *pxNetworkContext) {
  MQTTStatus_t xResult;
  MQTTConnectInfo_t xConnectInfo;
  bool xSessionPresent;
  TransportInterface_t xTransport;
  BaseType_t xStatus = pdFAIL;

  /* Fill in Transport Interface send and receive function pointers. */
  xTransport.pNetworkContext = pxNetworkContext;
  xTransport.send = SecureSocketsTransport_Send;
  xTransport.recv = SecureSocketsTransport_Recv;

  /* Initialize MQTT library. */
  xResult = MQTT_Init(pxMQTTContext, &xTransport, prvGetTimeMs, prvEventCallback, &xBuffer);
  configASSERT(xResult == MQTTSuccess);

  /* Some fields are not used in this demo so start with everything at 0. */
  (void)memset((void *)&xConnectInfo, 0x00, sizeof(xConnectInfo));

  /* Start with a clean session i.e. direct the MQTT broker to discard any
   * previous session data. Also, establishing a connection with clean session
   * will ensure that the broker does not store any data when this client
   * gets disconnected. */
  xConnectInfo.cleanSession = true;

  /* The client identifier is used to uniquely identify this MQTT client to
   * the MQTT broker. In a production device the identifier can be something
   * unique, such as a device serial number. */
  xConnectInfo.pClientIdentifier = democonfigCLIENT_IDENTIFIER;
  xConnectInfo.clientIdentifierLength = (uint16_t)strlen(democonfigCLIENT_IDENTIFIER);

  /* Use the metrics string as username to report the OS and MQTT client version
   * metrics to AWS IoT. */
  xConnectInfo.pUserName = AWS_IOT_METRICS_STRING;
  xConnectInfo.userNameLength = AWS_IOT_METRICS_STRING_LENGTH;

  /* Set MQTT keep-alive period. If the application does not send packets at an interval less than
   * the keep-alive period, the MQTT library will send PINGREQ packets. */
  xConnectInfo.keepAliveSeconds = mqttexampleKEEP_ALIVE_TIMEOUT_SECONDS;

  /* Send MQTT CONNECT packet to broker. LWT is not used in this demo, so it
   * is passed as NULL. */
  xResult = MQTT_Connect(pxMQTTContext, &xConnectInfo, NULL, mqttexampleCONNACK_RECV_TIMEOUT_MS,
                         &xSessionPresent);

  if (xResult != MQTTSuccess) {
    LogError(("Failed to establish MQTT connection: Server=%s, MQTTStatus=%s",
              democonfigMQTT_BROKER_ENDPOINT, MQTT_Status_strerror(xResult)));
  } else {
    /* Successfully established and MQTT connection with the broker. */
    LogInfo(("An MQTT connection is established with %s.", democonfigMQTT_BROKER_ENDPOINT));
    xStatus = pdPASS;
  }

  return xStatus;
}
/*-----------------------------------------------------------*/

static void prvUpdateSubAckStatus(MQTTPacketInfo_t *pxPacketInfo) {
  MQTTStatus_t xResult = MQTTSuccess;
  uint8_t *pucPayload = NULL;
  size_t ulSize = 0;
  uint32_t ulTopicCount = 0U;

  xResult = MQTT_GetSubAckStatusCodes(pxPacketInfo, &pucPayload, &ulSize);

  /* MQTT_GetSubAckStatusCodes always returns success if called with packet info
   * from the event callback and non-NULL parameters. */
  configASSERT(xResult == MQTTSuccess);

  for (ulTopicCount = 0; ulTopicCount < ulSize; ulTopicCount++) {
    xTopicFilterContext[ulTopicCount].xSubAckStatus = pucPayload[ulTopicCount];
  }
}

/*-----------------------------------------------------------*/

static BaseType_t prvMQTTPublishToTopic(MQTTContext_t *pxMQTTContext, const void *payload,
                                        size_t size) {
  MQTTStatus_t xResult;
  MQTTPublishInfo_t xMQTTPublishInfo;
  BaseType_t xStatus = pdPASS;

  /* Some fields are not used by this demo so start with everything at 0. */
  (void)memset((void *)&xMQTTPublishInfo, 0x00, sizeof(xMQTTPublishInfo));

  /* This demo uses QoS1. */
  xMQTTPublishInfo.qos = MQTTQoS1;
  xMQTTPublishInfo.retain = false;
  xMQTTPublishInfo.pTopicName = mqttexampleTOPIC;
  xMQTTPublishInfo.topicNameLength = (uint16_t)strlen(mqttexampleTOPIC);
  xMQTTPublishInfo.pPayload = payload;
  xMQTTPublishInfo.payloadLength = size;

  /* Get a unique packet id. */
  usPublishPacketIdentifier = MQTT_GetPacketId(pxMQTTContext);

  /* Send PUBLISH packet. Packet ID is not used for a QoS1 publish. */
  xResult = MQTT_Publish(pxMQTTContext, &xMQTTPublishInfo, usPublishPacketIdentifier);

  if (xResult != MQTTSuccess) {
    xStatus = pdFAIL;
    LogError(("Failed to send PUBLISH message to broker: Topic=%s, Error=%s", mqttexampleTOPIC,
              MQTT_Status_strerror(xResult)));
  }

  return xStatus;
}

/*-----------------------------------------------------------*/

static void prvMQTTProcessResponse(MQTTPacketInfo_t *pxIncomingPacket, uint16_t usPacketId) {
  uint32_t ulTopicCount = 0U;

  switch (pxIncomingPacket->type) {
    case MQTT_PACKET_TYPE_PUBACK:
      LogInfo(("PUBACK received for packet Id %u.", usPacketId));
      /* Make sure ACK packet identifier matches with Request packet identifier. */
      configASSERT(usPublishPacketIdentifier == usPacketId);
      break;

    case MQTT_PACKET_TYPE_SUBACK:

      /* Update the packet type received to SUBACK. */
      usPacketTypeReceived = MQTT_PACKET_TYPE_SUBACK;

      /* A SUBACK from the broker, containing the server response to our subscription request, has
       * been received. It contains the status code indicating server approval/rejection for the
       * subscription to the single topic requested. The SUBACK will be parsed to obtain the status
       * code, and this status code will be stored in global variable #xTopicFilterContext. */
      prvUpdateSubAckStatus(pxIncomingPacket);

      for (ulTopicCount = 0; ulTopicCount < mqttexampleTOPIC_COUNT; ulTopicCount++) {
        if (xTopicFilterContext[ulTopicCount].xSubAckStatus != MQTTSubAckFailure) {
          LogInfo(("Subscribed to the topic %s with maximum QoS %u.",
                   xTopicFilterContext[ulTopicCount].pcTopicFilter,
                   xTopicFilterContext[ulTopicCount].xSubAckStatus));
        }
      }

      /* Make sure ACK packet identifier matches with Request packet identifier. */
      configASSERT(usSubscribePacketIdentifier == usPacketId);
      break;

    case MQTT_PACKET_TYPE_UNSUBACK:
      LogInfo(("Unsubscribed from the topic %s.", mqttforwardTOPIC));

      /* Update the packet type received to UNSUBACK. */
      usPacketTypeReceived = MQTT_PACKET_TYPE_UNSUBACK;

      /* Make sure ACK packet identifier matches with Request packet identifier. */
      configASSERT(usUnsubscribePacketIdentifier == usPacketId);
      break;

    case MQTT_PACKET_TYPE_PINGRESP:
      LogInfo(("Ping Response successfully received."));
      break;

    case MQTT_PACKET_TYPE_PUBLISH:
      LogInfo(("Publish successfully received from " mqttforwardTOPIC));
      break;

    /* Any other packet type is invalid. */
    default:
      LogWarn(("prvMQTTProcessResponse() called with unknown packet type:(%02X).",
               pxIncomingPacket->type));
  }
}

/*-----------------------------------------------------------*/

static void prvMQTTProcessIncomingPublish(MQTTPublishInfo_t *pxPublishInfo) {
  configASSERT(pxPublishInfo != NULL);

  /* Set the global for indicating that an incoming publish is received. */
  usPacketTypeReceived = MQTT_PACKET_TYPE_PUBLISH;

  /* Process incoming Publish. */
  LogInfo(("Incoming QoS : %d\n", pxPublishInfo->qos));

  /* Verify the received publish is for the we have subscribed to. */
  if ((pxPublishInfo->topicNameLength == strlen(mqttforwardTOPIC)) &&
      (0 == strncmp(mqttforwardTOPIC, pxPublishInfo->pTopicName, pxPublishInfo->topicNameLength))) {
    LogInfo(("Incoming Publish Topic Name: %.*s matches subscribed topic."
             "Incoming Publish Message : %.*s",
             pxPublishInfo->topicNameLength, pxPublishInfo->pTopicName,
             pxPublishInfo->payloadLength, pxPublishInfo->pPayload));
  } else {
    LogInfo(("Incoming Publish Topic Name: %.*s does not match subscribed topic.",
             pxPublishInfo->topicNameLength, pxPublishInfo->pTopicName));
  }
}

/*-----------------------------------------------------------*/

static void prvEventCallback(MQTTContext_t *pxMQTTContext, MQTTPacketInfo_t *pxPacketInfo,
                             MQTTDeserializedInfo_t *pxDeserializedInfo) {
  /* The MQTT context is not used for this demo. */
  (void)pxMQTTContext;

  if ((pxPacketInfo->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH) {
    prvMQTTProcessIncomingPublish(pxDeserializedInfo->pPublishInfo);
  } else {
    prvMQTTProcessResponse(pxPacketInfo, pxDeserializedInfo->packetIdentifier);
  }
}

/*-----------------------------------------------------------*/

static uint32_t prvGetTimeMs(void) {
  TickType_t xTickCount = 0;
  uint32_t ulTimeMs = 0UL;

  /* Get the current tick count. */
  xTickCount = xTaskGetTickCount();

  /* Convert the ticks to milliseconds. */
  ulTimeMs = (uint32_t)xTickCount * MILLISECONDS_PER_TICK;

  /* Reduce ulGlobalEntryTimeMs from obtained time so as to always return the
   * elapsed time in the application. */
  ulTimeMs = (uint32_t)(ulTimeMs - ulGlobalEntryTimeMs);

  return ulTimeMs;
}

/*-----------------------------------------------------------*/
