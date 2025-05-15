/*ЗАГОЛОВОЧНЫЕ ФАЙЛЫ И ПРОСТРАНСТВА ИМЕН*/

#include <iostream>       // Для ввода/вывода
#include <map>           // Для использования std::map (хранение состояний пинов)
#include <cstdint>       // Для uint8_t (беззнаковый 8-битный тип)
#include <string>        // Для работы со строками
#include <cstring>       // Для работы с C-строками (используется в MQTT)
#include <mosquitto.h>   // Библиотека MQTT (mosquitto)
#include <nlohmann/json.hpp> // Для работы с JSON
#include <chrono>        // Для работы со временем (задержки, интервалы)
#include <thread>        // Для std::this_thread::sleep_for
#include <cstdlib>       // Для std::getenv (чтение переменных окружения)
#include <random>        // Для генерации случайных чисел (температура)
#include <ctime>         // Для std::time (инициализация генератора случайных чисел)

using json = nlohmann::json; // Упрощение работы с JSON



/*КОНСТАНТЫ И ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ*/

// Константы для задержек (в миллисекундах)
constexpr int MAIN_LOOP_DELAY = 100;    // 100ms задержка основного цикла
constexpr int MQTT_LOOP_DELAY = 10;     // 10ms задержка для обработки MQTT
constexpr int RECONNECT_DELAY = 5000;   // 5s задержка между попытками реконнекта
constexpr int MAX_RECONNECT_ATTEMPTS = 10; // Максимальное количество попыток реконнекта

// Номера пинов для RGB и датчика температуры
constexpr uint8_t RED_PIN = 3;       // Пин для красного цвета
constexpr uint8_t GREEN_PIN = 5;     // Пин для зеленого цвета
constexpr uint8_t BLUE_PIN = 6;      // Пин для синего цвета
constexpr uint8_t TEMPERATURE_PIN = A0; // Аналоговый пин для температуры (A0)

// Глобальные переменные
std::map<uint8_t, bool> pinStates;      // Состояния цифровых пинов (HIGH/LOW)
std::map<uint8_t, int> analogPinValues; // Состояния аналоговых пинов (например, A0)
struct mosquitto *mosq = nullptr;       // Указатель на MQTT-клиент
bool shouldRestart = false;             // Флаг перезагрузки
bool isConnected = false;               // Флаг подключения к MQTT
int reconnectAttempts = 0;              // Счетчик попыток переподключения
auto lastTemperaturePublishTime = std::chrono::steady_clock::now(); // Время последней публикации температуры

// Генератор случайных чисел (для температуры 20-30°C)
std::mt19937 rng(std::time(nullptr)); // Инициализация генератора
std::uniform_int_distribution<int> tempDist(20, 30); // Распределение 20-30

// Текущие значения ШИМ для яркости каждого пина (0-255)
std::map<uint8_t, uint8_t> pwmValues; 
// Время последнего изменения состояния для каждого пина
std::map<uint8_t, std::chrono::steady_clock::time_point> lastPwmToggleTime;
// Период ШИМ (в микросекундах)
constexpr uint16_t PWM_PERIOD = 1000; // 1 мс (частота ~1 кГц)





/*ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ*/
//Чтение переменных окружения
std::string getEnvVar(const char* name, const char* defaultValue) 
{
    const char* value = std::getenv(name); // Пытаемся прочитать переменную
    return value ? value : defaultValue;   // Если не найдена — возвращаем значение по умолчанию
}



 /*ФУНКЦИИ ДЛЯ РАБОТЫ С MQTT */

//Публикация ошибок в MQTT
void publishError(const std::string& errorMessage) 
{
    if (mosq && isConnected) // Если MQTT подключен
    { 
        json error;
        error["error"] = errorMessage; // Сообщение об ошибке
        error["timestamp"] = std::chrono::system_clock::now().time_since_epoch().count(); // Метка времени
        std::string payload = error.dump(); 
        mosquitto_publish(mosq, nullptr, "embedded/errors", payload.length(), payload.c_str(), 1, false); // QoS=1
    }
    std::cerr << "ERROR: " << errorMessage << std::endl; // Логирование в консоль
}

// Функция для подключения к брокеру MQTT
bool connectToMqtt() 
{
    // Чтение настроек из переменных окружения
    std::string mqttHost = getEnvVar("MQTT_HOST", "localhost");
    int mqttPort = std::stoi(getEnvVar("MQTT_PORT", "1883"));
    std::string mqttUsername = getEnvVar("MQTT_USERNAME", "");
    std::string mqttPassword = getEnvVar("MQTT_PASSWORD", "");
    std::cout << "Connecting to MQTT broker at " << mqttHost << ":" << mqttPort << std::endl;
    
    // Установка учетных данных
    if (!mqttUsername.empty() && !mqttPassword.empty()) 
    {
        mosquitto_username_pw_set(mosq, mqttUsername.c_str(), mqttPassword.c_str());
    }
    // Подключение к брокеру
    int result = mosquitto_connect(mosq, mqttHost.c_str(), mqttPort, 60);
    if (result != MOSQ_ERR_SUCCESS) 
    {
        publishError("Unable to connect to MQTT broker: " + std::string(mosquitto_strerror(result)));
        return false;
    }
    
    // Добавляем начальную синхронизацию после подключения
    mosquitto_loop(mosq, 100, 1);
    return true;
}





/*CALLBACKS*/

// Callback для подключения к MQTT
void connect_callback(struct mosquitto *mosq, void *obj, int result) 
{
    if (result == MOSQ_ERR_SUCCESS) 
    {
        std::cout << "Successfully connected to MQTT broker" << std::endl;
        isConnected = true;
        reconnectAttempts = 0;

        int rc = mosquitto_subscribe(mosq, nullptr, "embedded/control", 0);
        if (rc != MOSQ_ERR_SUCCESS) 
        {
            publishError("Failed to connect: " + std::string(mosquitto_strerror(result)));
        } 
        else 
        {
            std::cout << "Successfully subscribed to embedded/control" << std::endl;
        }
    } 
    else 
    {
        publishError("Failed to connect to MQTT broker: " + std::string(mosquitto_strerror(result)));
        isConnected = false;
    }
}

// Callback для отключения от MQTT
void disconnect_callback(struct mosquitto *mosq, void *obj, int result) 
{
    std::cout << "Disconnected from MQTT broker" << std::endl;
    isConnected = false;
}

// Callback для получения сообщений MQTT
void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) 
{
    if (!message->payload) 
    {
        std::cout << "Received empty message" << std::endl;
        return;
    }

    std::string topic(message->topic);
    std::string payload(static_cast<char*>(message->payload), message->payloadlen);
    
    std::cout << "Received message on topic: " << topic << ", payload: " << payload << std::endl;
    
    try 
    {
        json data = json::parse(payload);
        
        if (topic == "embedded/control") 
        {
            if (data.contains("command")) 
            {
                std::string command = data["command"];
                if (command == "restart") 
                {
                    std::cout << "Received restart command" << std::endl;
                    // Изменяем состояние пина 2 перед перезапуском
                    bool currentState = digitalRead(2);
                    digitalWrite(2, !currentState); // Инвертируем текущее состояние
                    shouldRestart = true;
                }
                else if (command == "set_rgb") // Валидация и установка RGB
                {
                    if (!data.contains("red") || !data.contains("green") || !data.contains("blue")) 
                    {
                        publishError("Invalid RGB command");
                        return;
                    }
                    int red = data["red"];
                    int green = data["green"];
                    int blue = data["blue"];
                    
                    if (red < 0 || red > 255 || green < 0 || green > 255 || blue < 0 || blue > 255) 
                    {
                        publishError("Invalid RGB values: values must be between 0 and 255");
                        return;
                    }
                    setRGB_digital(data["red"], data["green"], data["blue"]);
                }
                else 
                {
                    publishError("Unknown command received: " + command);
                }
            }
            else 
            {
                publishError("Received message without command field");
            }
        }
    } 
    catch (const std::exception& e)
    {
        publishError("JSON parse error: " + std::string(e.what()));
    }
}




/*ФУНКЦИИ ДЛЯ РАБОТЫ С GPIO*/
// ЦИФРОВЫЕ ПИНЫ
// Функция для установки режима пина (вход/выход)
void pinMode(uint8_t pin, bool isOutput) 
{
    std::cout << "Pin " << (int)pin << " set to " << (isOutput ? "OUTPUT" : "INPUT") << std::endl;
    pinStates[pin] = false; // Инициализация состояния пина
}

// Функция для чтения значения с пина. 
/*pinStates.end() возвращает итератор, указывающий на "конец" контейнера (несуществующий элемент)
Условие find(pin) == end() означает, что пин не найден в pinStates (т.е. не был инициализирован через pinMode)*/
bool digitalRead(uint8_t pin)
{
    if (pinStates.find(pin) == pinStates.end()) 
    {
        publishError("Attempt to read from uninitialized pin: " + std::to_string(pin));
        return false;
    }
    std::cout << "Reading from pin " << (int)pin << ": " << (pinStates[pin] ? "HIGH" : "LOW") << std::endl;
    return pinStates[pin];
}

// Функция для записи значения на пин
void digitalWrite(uint8_t pin, bool value) 
{
    if (pinStates.find(pin) == pinStates.end()) 
    {
        publishError("Attempt to write to uninitialized pin: " + std::to_string(pin));
        return;
    }
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
            if (rc == MOSQ_ERR_SUCCESS) 
            {
                std::cout << "Successfully published MQTT message" << std::endl;
                // Важно: нужно вызвать mosquitto_loop для обработки исходящих сообщений
                mosquitto_loop(mosq, 100, 1); // Даем время на обработку сообщения
                break;
            }
            else if (rc == MOSQ_ERR_NO_CONN) 
            {
                publishError("No connection to broker, attempting to reconnect...");
                if (connectToMqtt()) 
                {
                    mosquitto_loop(mosq, 100, 1);
                }
            } 
            else
            {
                publishError("Failed to publish MQTT message:" + std::string(mosquitto_strerror(rc)));
            }
            retries--;
            if (retries > 0) {
                std::cout << "Retrying publish... (" << retries << " attempts left)" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
    }
}


// АНАЛОГОВЫЕ ПИНЫ
// Функция чтения значения с пина
int analogRead(uint8_t pin) 
{
    if (analogPinValues.find(pin) == analogPinValues.end()) 
    {
        publishError("Attempt to read from uninitialized analog pin: " + std::to_string(pin));
        return 0;
    }
    return analogPinValues[pin]; // Возвращаем значение (например, температуру)
}

// Функция для записи значения яркости светодиода
void analogWrite(uint8_t pin, uint8_t value) 
{
    if (pin != RED_PIN && pin != GREEN_PIN && pin != BLUE_PIN)
    {
        publishError("analogWrite: pin " + std::to_string(pin) + " does not support PWM");
        return;
    }

    if (value > 255) 
    {
        publishError("analogWrite: value " + std::to_string(value) + " out of range (0-255)");
        value = 255; // Ограничиваем максимумом
    }

    pwmValues[pin] = value; // Сохраняем значение
    lastPwmToggleTime[pin] = std::chrono::steady_clock::now(); // Сбрасываем таймер
}



// УПРАВЛЕНИЕ RGB И ТЕМПЕРАТУРОЙ
// фУНКЦИЯ ЕСЛИ НЕОБХОДИМО ВСЕГО ДВА СОСТОЯНИЯ СВЕТОДИОДА - ВКЛ\ВЫКЛ
void setRGB_digital(uint8_t red, uint8_t green, uint8_t blue) 
{
    // если значение >127 — включаем пин
    digitalWrite(RED_PIN, red > 127);
    digitalWrite(GREEN_PIN, green > 127);
    digitalWrite(BLUE_PIN, blue > 127);
    
    // Публикация подтверждения
    if (mosq && isConnected) 
    {
        json message;
        message["command"] = "set_rgb";
        message["red"] = red;
        message["green"] = green;
        message["blue"] = blue;
        mosquitto_publish(mosq, nullptr, "embedded/pins/state", message.dump().length(), message.dump().c_str(), 1, false);
    }
}

// ФУНКЦИЯ НА СЛУЧАЙ НЕОБХОДИМОСТИ УПРАВЛЕНИЯ ЯРКОСТЬЮ СВЕТОДИОДА  
void setRGB_analog(uint8_t red, uint8_t green, uint8_t blue) 
{
    analogWrite(RED_PIN, red );
    analogWrite(GREEN_PIN, green );
    analogWrite(BLUE_PIN, blue);
    
    // Публикация подтверждения
    if (mosq && isConnected) 
    {
        json message;
        message["command"] = "set_rgb";
        message["red"] = red;
        message["green"] = green;
        message["blue"] = blue;
        mosquitto_publish(mosq, nullptr, "embedded/pins/state", message.dump().length(), message.dump().c_str(), 1, false);
    }
}


// ПУБЛИКАЦИЯ ТЕМПЕРАТУРЫ
void publishTemperature() 
{
    int temperature = tempDist(rng); // Генерация случайной температуры (20-30°C)
    analogPinValues[TEMPERATURE_PIN] = temperature; // Сохраняем значение
    
    if (mosq && isConnected) 
    {
        json message;
        message["pin"] = TEMPERATURE_PIN;
        message["temperature"] = temperature;
        mosquitto_publish(mosq, nullptr, "embedded/sensors/temperature", message.dump().length(), message.dump().c_str(), 1, false);
    }
}



// ФУНКЦИЯ SETUP
void setup() 
{
    std::cout << "Setup started" << std::endl;
    
    mosquitto_lib_init();// Инициализация MQTT
    mosq = mosquitto_new("embedded-controller", true, nullptr);// Создание клиента
    if (!mosq)
    {
        publishError("Error: Out of memory." );
        return;
    }
    
    // Установка callback'ов
    mosquitto_connect_callback_set(mosq, connect_callback);
    mosquitto_disconnect_callback_set(mosq, disconnect_callback);
    mosquitto_message_callback_set(mosq, message_callback);
    
    // Настройка пинов
    pinMode(13, true);  // Пин 13 как выход
    pinMode(2, false);  // Пин 2 как вход
    // Настройка пинов RGB светодиода
    pinMode(RED_PIN, true);   
    pinMode(GREEN_PIN, true);
    pinMode(BLUE_PIN, true);
    
    // Попытка первоначального подключения
    if (connectToMqtt()) 
    {
        std::cout << "Initial MQTT connection successful" << std::endl;
    }
    
    std::cout << "Setup completed" << std::endl;
}


// LOOP
void loop() 
{
    static bool ledState = false;
    static auto lastMqttTime = std::chrono::steady_clock::now();
    static auto lastReconnectAttempt = std::chrono::steady_clock::now();
    
    if (!isConnected )  // Проверка подключения и попытка реконнекта
    {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReconnectAttempt).count() >= RECONNECT_DELAY) 
        {
            if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) 
            {
                std::cout << "Attempting to reconnect to MQTT broker (attempt " << (reconnectAttempts + 1) << ")" << std::endl;
                if (connectToMqtt()) 
                {
                    lastReconnectAttempt = now;
                    reconnectAttempts++;
                }
            } 
            else 
            {
                 publishError("Max reconnection attempts reached. Giving up." );
            }
        }
    }
    
    // Обработка MQTT сообщений с задержкой
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastMqttTime).count() >= MQTT_LOOP_DELAY) 
    {
        int rc = mosquitto_loop(mosq, 0, 1);
        if (rc != MOSQ_ERR_SUCCESS) 
        {
            publishError("MQTT loop error:" + std::string(mosquitto_strerror(rc)));
            isConnected = false;
        }
        lastMqttTime = now;
    }
    
    // Публикация температуры каждые 5 секунд
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTemperaturePublishTime).count() >= TEMPERATURE_PUBLISH_INTERVAL) 
    {
        publishTemperature();
        lastTemperaturePublishTime = now;
    }
    
    // Если получена команда перезапуска
    if (shouldRestart) 
    {
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
    if (buttonState) 
    {
        ledState = !ledState;
        digitalWrite(13, ledState);
    }

    // Задержка основного цикла
    std::this_thread::sleep_for(std::chrono::milliseconds(MAIN_LOOP_DELAY));
}

int main() 
{
    setup();
    
    // Эмуляция бесконечного цикла
    while (true) 
    {
        loop();
    }
    
    // Очистка MQTT
    if (mosq) 
    {
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
    }
    mosquitto_lib_cleanup();
    
    return 0;
} 
