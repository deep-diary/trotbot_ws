#ifndef DEEP_MOTOR_CONTROL_H
#define DEEP_MOTOR_CONTROL_H

#include "mcp_server.h"
#include "deep_motor.h"

/**
 * 电机子模块 MCP 工具注册，由板级 InitializeTools() 调用。
 * 不依赖 DeepMotorControl 实例，避免板级主文件臃肿。
 */
void RegisterMotorMcpTools(McpServer& mcp_server, DeepMotor* deep_motor);

class DeepMotorControl {
private:
    DeepMotor* deep_motor_;

public:
    explicit DeepMotorControl(DeepMotor* deep_motor, McpServer& mcp_server);
    ~DeepMotorControl() = default;
};

#endif // DEEP_MOTOR_CONTROL_H
