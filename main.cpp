#include <iostream>
#include <map>
#include <cstdint>
#include <string>
#include <cstring>
#include <mosquitto.h>
#include <nlohmann/json.hpp>

#include <chrono>
#include <thread>
#include <cstdlib>

using json = nlohmann::json;

// Константы для задержек (в миллисекундах)
constexpr int MAIN_LOOP_DELAY = 100;    // 100ms задержка основного цикла
constexpr int MQTT_LOOP_DELAY = 10;     // 10ms задержка для обработки MQTT
constexpr int RECONNECT_DELAY = 5000;   // 5s задержка между попытками реконнекта
constexpr int MAX_RECONNECT_ATTEMPTS = 10; // Максимальное количество попыток реконнекта

// Эмуляция состояния пинов
std::map<uint8_t, bool> pinStates;
struct mosquitto *mosq = nullptr;
bool shouldRestart = false;
bool isConnected = false;
int reconnectAttempts = 0;

// Получение переменных окружения с значениями по умолчанию
std::string getEnvVar(const char* name, const char* defaultValue) {
    const char* value = std::getenv(name);
    return value ? value : defaultValue;
}

// Функция для подключения к MQTT
bool connectToMqtt() {
    // Получение учетных данных из переменных окружения
    std::string mqttHost = getEnvVar("MQTT_HOST", "localhost");
    int mqttPort = std::stoi(getEnvVar("MQTT_PORT", "1883"));
    std::string mqttUsername = getEnvVar("MQTT_USERNAME", "");
    std::string mqttPassword = getEnvVar("MQTT_PASSWORD", "");
    
    std::cout << "Connecting to MQTT broker at " << mqttHost << ":" << mqttPort << std::endl;
    
    // Установка учетных данных
    if (!mqttUsername.empty() && !mqttPassword.empty()) {
        mosquitto_username_pw_set(mosq, mqttUsername.c_str(), mqttPassword.c_str());
    }
    
    // Подключение к брокеру
    int result = mosquitto_connect(mosq, mqttHost.c_str(), mqttPort, 60);
    if (result != MOSQ_ERR_SUCCESS) {
        std::cerr << "Unable to connect to MQTT broker: " << mosquitto_strerror(result) << std::endl;
        return false;
    }
    
    // Добавляем начальную синхронизацию после подключения
    mosquitto_loop(mosq, 100, 1);
    
    return true;
}

// Callback для подключения к MQTT
void connect_callback(struct mosquitto *mosq, void *obj, int result) {
    if (result == MOSQ_ERR_SUCCESS) {
        std::cout << "Successfully connected to MQTT broker" << std::endl;
        isConnected = true;
        reconnectAttempts = 0;
        
        // Подписываемся на топик после подключения
        int rc = mosquitto_subscribe(mosq, nullptr, "embedded/control", 0);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "Failed to subscribe to topic: " << mosquitto_strerror(rc) << std::endl;
        } else {
            std::cout << "Successfully subscribed to embedded/control" << std::endl;
        }
    } else {
        std::cerr << "Failed to connect to MQTT broker: " << mosquitto_strerror(result) << std::endl;
        isConnected = false;
    }
}

// Callback для отключения от MQTT
void disconnect_callback(struct mosquitto *mosq, void *obj, int result) {
    std::cout << "Disconnected from MQTT broker" << std::endl;
    isConnected = false;
}

// Функция для установки режима пина (вход/выход)
void pinMode(uint8_t pin, bool isOutput) {
    std::cout << "Pin " << (int)pin << " set to " << (isOutput ? "OUTPUT" : "INPUT") << std::endl;
    pinStates[pin] = false; // Инициализация состояния пина
}

// Функция для чтения значения с пина
bool digitalRead(uint8_t pin) {
    std::cout << "Reading from pin " << (int)pin << ": " << (pinStates[pin] ? "HIGH" : "LOW") << std::endl;
    return pinStates[pin];
}

// Функция для записи значения на пин
void digitalWrite(uint8_t pin, bool value) {
    std::cout << "Writing to pin " << (int)pin << ": " << (value ? "HIGH" : "LOW") << std::endl;
    pinStates[pin] = value;
    
    // Отправляем состояние пина в MQTT только если подключены
    if (mosq && isConnected) {
        json message;
        message["pin"] = pin;
        message["value"] = value;
        std::string payload = message.dump();
        
        std::cout << "Publishing MQTT message to topic 'embedded/pins/state': " << payload << std::endl;
        
        // Добавляем обработку ошибок и повторные попытки публикации
        int retries = 3;
        while (retries > 0) {
            int rc = mosquitto_publish(mosq, nullptr, "embedded/pins/state", payload.length(), payload.c_str(), 1, false); // QoS=1 для гарантированной доставки
            if (rc == MOSQ_ERR_SUCCESS) {
                std::cout << "Successfully published MQTT message" << std::endl;
                
                // Важно: нужно вызвать mosquitto_loop для обработки исходящих сообщений
                mosquitto_loop(mosq, 100, 1); // Даем время на обработку сообщения
                break;
            } else if (rc == MOSQ_ERR_NO_CONN) {
                std::cerr << "No connection to broker, attempting to reconnect..." << std::endl;
                if (connectToMqtt()) {
                    mosquitto_loop(mosq, 100, 1);
                }
            } else {
                std::cerr << "Failed to publish MQTT message: " << mosquitto_strerror(rc) << std::endl;
            }
            retries--;
            if (retries > 0) {
                std::cout << "Retrying publish... (" << retries << " attempts left)" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
}

// Callback для получения сообщений MQTT
void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {
    if (!message->payload) {
        std::cout << "Received empty message" << std::endl;
        return;
    }

    std::string topic(message->topic);
    std::string payload(static_cast<char*>(message->payload), message->payloadlen);
    
    std::cout << "Received message on topic: " << topic << ", payload: " << payload << std::endl;
    
    try {
        json data = json::parse(payload);
        
        if (topic == "embedded/control") {
            if (data.contains("command")) {
                std::string command = data["command"];
                if (command == "restart") {
                    std::cout << "Received restart command" << std::endl;
                    // Изменяем состояние пина 2 перед перезапуском
                    bool currentState = digitalRead(2);
                    digitalWrite(2, !currentState); // Инвертируем текущее состояние
                    shouldRestart = true;
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
    }
}

// Функция setup - выполняется один раз при старте
void setup() {
    std::cout << "Setup started" << std::endl;
    
    // Инициализация MQTT
    mosquitto_lib_init();
    mosq = mosquitto_new("embedded-controller", true, nullptr);
    if (!mosq) {
        std::cerr << "Error: Out of memory." << std::endl;
        return;
    }
    
    // Установка callback'ов
    mosquitto_connect_callback_set(mosq, connect_callback);
    mosquitto_disconnect_callback_set(mosq, disconnect_callback);
    mosquitto_message_callback_set(mosq, message_callback);
    
    // Настройка пинов
    pinMode(13, true);  // Пин 13 как выход
    pinMode(2, false);  // Пин 2 как вход
    
    // Попытка первоначального подключения
    if (connectToMqtt()) {
        std::cout << "Initial MQTT connection successful" << std::endl;
    }
    
    std::cout << "Setup completed" << std::endl;
}

// Функция loop - выполняется циклически
void loop() {
    static bool ledState = false;
    static auto lastMqttTime = std::chrono::steady_clock::now();
    static auto lastReconnectAttempt = std::chrono::steady_clock::now();
    
    // Проверка подключения и попытка реконнекта
    if (!isConnected) {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReconnectAttempt).count() >= RECONNECT_DELAY) {
            if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
                std::cout << "Attempting to reconnect to MQTT broker (attempt " << (reconnectAttempts + 1) << ")" << std::endl;
                if (connectToMqtt()) {
                    lastReconnectAttempt = now;
                    reconnectAttempts++;
                }
            } else {
                std::cerr << "Max reconnection attempts reached. Giving up." << std::endl;
            }
        }
    }
    
    // Обработка MQTT сообщений с задержкой
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastMqttTime).count() >= MQTT_LOOP_DELAY) {
        int rc = mosquitto_loop(mosq, 0, 1);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "MQTT loop error: " << mosquitto_strerror(rc) << std::endl;
            isConnected = false;
        }
        lastMqttTime = now;
    }
    
    // Если получена команда перезапуска
    if (shouldRestart) {
        std::cout << "Restarting..." << std::endl;
        std::cout << "Waiting 3 seconds before restart..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        shouldRestart = false;
        setup(); // Перезапускаем setup
        return;
    }
    
    // Чтение значения с пина 2
    bool buttonState = digitalRead(2);
    
    // Если кнопка нажата (пин 2 в HIGH), переключаем светодиод
    if (buttonState) {
        ledState = !ledState;
        digitalWrite(13, ledState);
    }
    
    // Задержка основного цикла
    std::this_thread::sleep_for(std::chrono::milliseconds(MAIN_LOOP_DELAY));
}

int main() {
    setup();
    
    // Эмуляция бесконечного цикла
    while (true) {
        loop();
    }
    
    // Очистка MQTT
    if (mosq) {
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
    }
    mosquitto_lib_cleanup();
    
    return 0;
} 
