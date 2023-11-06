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
#include <errno.h>  // Agrega esta inclusión
#include <fcntl.h>  // Agrega esta inclusión
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <future>
#include <iostream>
#include <smacc2/client_bases/smacc_ros_launch_client_2.hpp>
#include <thread>

namespace smacc2
{
namespace client_bases
{
using namespace std::chrono_literals;
ClRosLaunch2::ClRosLaunch2(/*std::string packageName, std::string launchFilename*/)
: /*packageName_(std::nullopt), launchFileName_(std::nullopt),*/ cancellationToken_(false)
{
}

ClRosLaunch2::ClRosLaunch2(std::string packageName, std::string launchFilename)
: packageName_(packageName), launchFileName_(launchFilename), cancellationToken_(false)
{
}

ClRosLaunch2::~ClRosLaunch2() {}

void ClRosLaunch2::launch()
{
  cancellationToken_.store(false);
  // Iniciar el hilo para la ejecución del lanzamiento
  this->result_ = /*std::async([this]()*/
                  // {
    executeRosLaunch(packageName_, launchFileName_, [this]() { return cancellationToken_.load(); });
  // });
}

void ClRosLaunch2::stop()
{
  // Establecer la bandera de cancelación
  cancellationToken_.store(true);
}

std::future<std::string> ClRosLaunch2::executeRosLaunch(
  std::string packageName, std::string launchFileName, std::function<bool()> cancelCondition,
  ClRosLaunch2 * client)
// std::string ClRosLaunch2::executeRosLaunch(std::string packageName, std::string launchFileName, std::function<bool()> cancelCondition)
{
  return std::async(
    std::launch::async,
    [packageName, launchFileName, cancelCondition, client]()
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("smacc2"), "[ClRosLaunch2] Starting ros launch thread");

      std::stringstream cmd;
      cmd << "ros2 launch " << packageName << " " << launchFileName;
      std::array<char, 128> buffer;
      std::string result;

      auto child = runProcess(cmd.str().c_str());

      if (!child.pipe)
      {
        throw std::runtime_error("popen() failed!");
      }
      if (client != nullptr)
      {
        client->launchPid_ = child.pid;
      }

      int fd = fileno(child.pipe);

      int flags = fcntl(fd, F_GETFL, 0);
      fcntl(fd, F_SETFL, flags | O_NONBLOCK);

      bool cancelled = false;

      // while (!cancelCondition())
      while (!cancelled)
      {
        cancelled = cancelCondition();
        size_t bytesRead = fread(buffer.data(), 1, buffer.size(), /*data*/ child.pipe);

        if (bytesRead > 0)
        {
          result.append(buffer.data(), bytesRead);
        }
        else if (bytesRead == 0)
        {
          // No se han leído más datos
          std::this_thread::sleep_for(
            std::chrono::milliseconds(100));  // Espera antes de intentar nuevamente
        }
        else
        {
          // Error de lectura
          RCLCPP_ERROR(rclcpp::get_logger("smacc2"), "Error de lectura en pipe");
          break;
        }
      }

      rclcpp::sleep_for(2s);
      if (child.pid > 0)
      {
        killGrandchildren(child.pid);
        rclcpp::sleep_for(2s);
      }

      int status;
      pid_t child_pid = child.pid;
      if (waitpid(child_pid, &status, 0) != -1)
      {
        if (WIFEXITED(status))
        {
          int exit_status = WEXITSTATUS(status);
          RCLCPP_INFO(
            rclcpp::get_logger("smacc2"), "Child process exited with status: %d", exit_status);
        }
        else if (WIFSIGNALED(status))
        {
          int term_signal = WTERMSIG(status);
          RCLCPP_WARN(
            rclcpp::get_logger("smacc2"), "Child process terminated by signal: %d", term_signal);
        }
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger("smacc2"), "Error waiting for child process.");
      }

      pclose(child.pipe);
      close(child.pipe->_fileno);  // Close pipe file descriptor but not processes

      RCLCPP_WARN_STREAM(rclcpp::get_logger("smacc2"), "[ClRosLaunch2] RESULT:\n" << result);

      // Devuelve una std::future con el resultado
      // return std::async(std::launch::async, [result]() { return result; });
      return result;
    });
}

/**Aditional functions**/
/////////////////////////////////////////////////////

ProcessInfo runProcess(const char * command)
{
  ProcessInfo info;
  info.pid = -1;  // Inicializar el PID a -1 (indicando error)
  info.pipe = nullptr;

  int pipefd[2];  // Descriptor de archivo para el pipe

  if (pipe(pipefd) == -1)
  {
    perror("pipe");
    return info;
  }

  pid_t pid = fork();

  if (pid == 0)
  {
    // Esto se ejecuta en el proceso hijo
    close(pipefd[0]);                // Cerramos el extremo de lectura del pipe en el proceso hijo
    dup2(pipefd[1], STDOUT_FILENO);  // Redireccionamos la salida estándar al pipe
    close(pipefd[1]);                // Cerramos el extremo de escritura del pipe en el proceso hijo

    execl("/bin/sh", "/bin/sh", "-c", command, nullptr);  // Ejecuta el comando dado

    // Si execl retorna, significa que hubo un error
    std::cerr << "Error al ejecutar el comando: " << command << std::endl;
    _exit(1);  // Usar _exit para evitar la ejecución de códigos de salida de manejo de errores
  }
  else if (pid > 0)
  {
    // Esto se ejecuta en el proceso padre
    close(pipefd[1]);  // Cerramos el extremo de escritura del pipe en el proceso padre
    info.pid = pid;
    info.pipe = fdopen(pipefd[0], "r");  // Abrir el descriptor de archivo de lectura en modo texto
  }
  else
  {
    std::cerr << "Error al crear el proceso hijo." << std::endl;
  }

  return info;
}

void killProcessesRecursive(pid_t pid)
{
  std::string command = "pgrep -P " + std::to_string(pid);
  FILE * pipe = popen(command.c_str(), "r");

  if (!pipe)
  {
    std::cerr << "Error executing pgrep command." << std::endl;
    return;
  }

  char buffer[128];
  std::string result = "";

  while (fgets(buffer, sizeof(buffer), pipe) != NULL)
  {
    result += buffer;
  }

  pclose(pipe);

  std::istringstream iss(result);
  pid_t childPid;
  std::vector<pid_t> childPids;

  while (iss >> childPid)
  {
    childPids.push_back(childPid);
  }

  // Kill child processes before killing the parent process
  for (const pid_t & child : childPids)
  {
    killProcessesRecursive(child);
  }

  // After killing all children, kill the parent process
  int res = kill(pid, SIGTERM);
  if (res == 0)
  {
    RCLCPP_FATAL(rclcpp::get_logger("smacc2"), "Killed process %d", pid);
  }
}

void killGrandchildren(pid_t originalPid) { killProcessesRecursive(originalPid); }
}  // namespace client_bases
}  // namespace smacc2

/*=============DOCUMENTATION=================*/

/**************BASH VERSION*******************/
/*************RECURSIVE KILL******************/
// #!/bin/bash

// # PID del proceso original (cambia esto al PID que desees)
// original_pid=1119572

// # Función recursiva para matar a los nietos
// kill_grandchildren() {
//   local parent_pid=$1
//   local children=$(pgrep -P $parent_pid)

//   for child in $children; do
//     # Matar al hijo (nieto del proceso original)
//     kill -9 $child

//     # Llamar a la función de manera recursiva para matar a los nietos
//     kill_grandchildren $child
//   done
// }

// # Llamar a la función para matar a los nietos del proceso original
// kill_grandchildren $original_pid
