// Copyright 2021 RobosoftAI Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/
#include <smacc2/client_bases/smacc_ros_launch_client_2.hpp>
#include <iostream>
#include <future>
#include <thread>
#include <cstring>
#include <cstdio>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <cstdlib>
#include <poll.h>
#include <fcntl.h>  // Agrega esta inclusión
#include <errno.h>  // Agrega esta inclusión

namespace smacc2
{
namespace client_bases
{
using namespace std::chrono_literals;
ClRosLaunch2::ClRosLaunch2(std::string packageName, std::string launchFilename)
    : packageName_(packageName), launchFileName_(launchFilename), cancellationToken_(false)
{
}

ClRosLaunch2::~ClRosLaunch2() {}

void ClRosLaunch2::launch()
{
    // Iniciar el hilo para la ejecución del lanzamiento
    result_ = executeRosLaunch(packageName_, launchFileName_, [this]() { return cancellationToken_.load(); });
}

void ClRosLaunch2::stop()
{
    // Establecer la bandera de cancelación
    cancellationToken_.store(true);
}



// std::future<std::string> ClRosLaunch2::executeRosLaunch(std::string packageName, std::string launchFileName, std::function<bool()> cancelCondition)
std::string ClRosLaunch2::executeRosLaunch(std::string packageName, std::string launchFileName, std::function<bool()> cancelCondition)
{
    RCLCPP_WARN_STREAM(rclcpp::get_logger("smacc2"), "[ClRosLaunch2] Starting ros launch thread");

    std::stringstream cmd;
    cmd << "ros2 launch " << packageName << " " << launchFileName;

    std::array<char, 128> buffer;
    std::string result;
    // std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.str().c_str(), "r"), pclose);

    auto child = runProcess(cmd.str().c_str());

    if (!child.pipe)
    {
        throw std::runtime_error("popen() failed!");
    }

    int fd = fileno(child.pipe);

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    bool cancelled = false;

    // while (!cancelCondition())
    while (!cancelled)
    {   cancelled = cancelCondition();
        size_t bytesRead = fread(buffer.data(), 1, buffer.size(), /*data*/child.pipe);

        if (bytesRead > 0)
        {
            result.append(buffer.data(), bytesRead);
        }
        else if (bytesRead == 0)
        {
            // No se han leído más datos
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Espera antes de intentar nuevamente
        }
        else
        {
            // Error de lectura
            RCLCPP_ERROR(rclcpp::get_logger("smacc2"), "Error de lectura en pipe");
            break;
        }
    }
    
    rclcpp::sleep_for(2s);
    if (child.pid > 0) {
        killGrandchildren(child.pid);
        rclcpp::sleep_for(2s);
        kill(child.pid, SIGKILL);
    }
    // pclose(pipe.get());
    close(child.pipe->_fileno); // Cierra fichero pipe pero no los procesos

    RCLCPP_WARN_STREAM(rclcpp::get_logger("smacc2"), "[ClRosLaunch2] RESULT:\n" << result);

    // Devuelve una std::future con el resultado
    // return std::async(std::launch::async, [result]() { return result; });
    return  result;
}


ProcessInfo runProcess(const char* command) {
    ProcessInfo info;
    info.pid = -1; // Inicializar el PID a -1 (indicando error)
    info.pipe = nullptr;

    int pipefd[2]; // Descriptor de archivo para el pipe

    if (pipe(pipefd) == -1) {
        perror("pipe");
        return info;
    }

    pid_t pid = fork();

    if (pid == 0) {
        // Esto se ejecuta en el proceso hijo
        close(pipefd[0]); // Cerramos el extremo de lectura del pipe en el proceso hijo
        dup2(pipefd[1], STDOUT_FILENO); // Redireccionamos la salida estándar al pipe
        close(pipefd[1]); // Cerramos el extremo de escritura del pipe en el proceso hijo

        execl("/bin/sh", "/bin/sh", "-c", command, nullptr); // Ejecuta el comando dado

        // Si execl retorna, significa que hubo un error
        std::cerr << "Error al ejecutar el comando: " << command << std::endl;
        _exit(1); // Usar _exit para evitar la ejecución de códigos de salida de manejo de errores
    } else if (pid > 0) {
        // Esto se ejecuta en el proceso padre
        close(pipefd[1]); // Cerramos el extremo de escritura del pipe en el proceso padre
        info.pid = pid;
        info.pipe = fdopen(pipefd[0], "r"); // Abrir el descriptor de archivo de lectura en modo texto
    } else {
        std::cerr << "Error al crear el proceso hijo." << std::endl;
    }

    return info;
}

void killGrandchildren(pid_t originalPid) {
    std::string command = "pgrep -P " + std::to_string(originalPid);
    FILE* pipe = popen(command.c_str(), "r");
    
    if (!pipe) {
        std::cerr << "Error al ejecutar el comando pgrep." << std::endl;
        return;
    }
    
    char buffer[128];
    std::string result = "";
    
    while (fgets(buffer, sizeof(buffer), pipe) != NULL) {
        result += buffer;
    }

    RCLCPP_FATAL(rclcpp::get_logger("smacc2"), "List of processes to kill:\n %s", result.c_str());

    pclose(pipe);
    
    std::istringstream iss(result);
    pid_t childPid;
    
    while (iss >> childPid) {
        kill(childPid, SIGKILL);
        RCLCPP_FATAL(rclcpp::get_logger("smacc2"), "Kill signal sended to process %s", std::to_string(static_cast<uint64_t>(childPid)).c_str());
    }
}

}  // namespace client_bases
}  // namespace smacc2




