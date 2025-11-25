# HTTP Client (http_client)

SMACC2 client library for making HTTP/HTTPS requests using Boost.Beast.

## Architecture

The `http_client` follows a pure component-based architecture with three specialized components:

```
ClHttp (Client - Orchestrator)
├── CpHttpConnectionManager - io_context and thread management
├── CpHttpSessionManager - SSL context and session creation
└── CpHttpRequestExecutor - Request execution and response handling
```

### Components

#### CpHttpConnectionManager
**Responsibility**: Manages the asynchronous I/O infrastructure

- Owns boost::asio::io_context
- Manages worker thread lifecycle
- Provides strand for thread-safe operations

**Lifetime**: State machine scoped (created with ClHttp, destroyed with state machine)

#### CpHttpSessionManager
**Responsibility**: Manages HTTP/HTTPS session configuration

- Parses server URLs (protocol, host, port)
- Manages SSL context for HTTPS connections
- Creates session objects (http_session or ssl_http_session)

**Lifetime**: State machine scoped

#### CpHttpRequestExecutor
**Responsibility**: Coordinates HTTP request execution

- Accepts request parameters (method, path, body, headers)
- Acquires strand from CpHttpConnectionManager
- Acquires session from CpHttpSessionManager
- Executes requests and emits response signals

**Lifetime**: State machine scoped

### Signal Flow

```
User Code (Behavior)
    ↓
CpHttpRequestExecutor::executeRequest()
    ↓
Session created (http_session/ssl_http_session)
    ↓
Request executed asynchronously
    ↓
Response received
    ↓
CpHttpRequestExecutor::onResponseReceived_ (SmaccSignal)
    ↓ (backward compatibility)
ClHttp::onResponseReceived_ (forwarded)
    ↓
Behavior callback
```

## Usage

### Quick Start: Using Base Behaviors

The easiest way to make HTTP requests is using the provided base behaviors:

```cpp
#include <http_client/client_behaviors/cb_http_get_request.hpp>

class StHttpRequest : public smacc2::SmaccState<StHttpRequest, SmExample>
{
public:
  using SmaccState::SmaccState;

  void runtimeConfigure() override
  {
    auto behavior = this->configure<cl_http::CbHttpGetRequest>();
  }
};
```

### Custom Behaviors: Component-Based Approach

For custom HTTP behavior, inherit from base class and override response handling:

```cpp
#include <http_client/client_behaviors/cb_http_request.hpp>

class CbCustomRequest : public cl_http::CbHttpRequestBase
{
public:
  CbCustomRequest() : CbHttpRequestBase(CpHttpRequestExecutor::HttpMethod::GET) {}

  void onResponseReceived(const CpHttpRequestExecutor::TResponse& response) override
  {
    if (response.result() == boost::beast::http::status::ok)
    {
      RCLCPP_INFO(getLogger(), "Success: %s", response.body().c_str());
    }
  }
};
```

### Advanced: Direct Component Access

For full control, access the executor component directly:

```cpp
class CbAdvancedHttp : public smacc2::SmaccClientBehavior
{
public:
  void runtimeConfigure() override
  {
    this->requiresComponent(requestExecutor_);
    this->getStateMachine()->createSignalConnection(
      requestExecutor_->onResponseReceived_,
      &CbAdvancedHttp::onResponse,
      this
    );
  }

  void onEntry() override
  {
    std::unordered_map<std::string, std::string> headers = {
      {"Accept", "application/json"},
      {"Authorization", "Bearer token123"}
    };

    requestExecutor_->executeRequest(
      CpHttpRequestExecutor::HttpMethod::POST,
      "/api/data",
      R"({"key": "value"})",  // JSON body
      headers
    );
  }

  void onResponse(const CpHttpRequestExecutor::TResponse& response)
  {
    RCLCPP_INFO(
      getLogger(),
      "HTTP %d: %s",
      response.result_int(),
      response.body().c_str()
    );
  }

private:
  CpHttpRequestExecutor* requestExecutor_;
};
```

## State Machine Configuration

### Orthogonal Setup

```cpp
namespace sm_example
{
namespace clients
{
class ClHttpOrthogonal : public smacc2::Orthogonal<ClHttpOrthogonal>
{
public:
  void onInitialize() override
  {
    auto httpClient = this->createClient<cl_http::ClHttp>(
      "https://example.com",  // Server URL
      1500                     // Timeout (ms)
    );
  }
};
}
}
```

### Server URL Formats

The client automatically detects HTTP vs HTTPS based on the URL:

```cpp
// HTTPS (SSL enabled)
this->createClient<cl_http::ClHttp>("https://api.example.com");

// HTTP (no SSL)
this->createClient<cl_http::ClHttp>("http://api.example.com");

// Custom port
this->createClient<cl_http::ClHttp>("https://api.example.com:8443");
```

## Supported HTTP Methods

- **GET**: Retrieve data
- **POST**: Submit data
- **PUT**: Update/replace data

```cpp
// Using enum
CpHttpRequestExecutor::HttpMethod::GET
CpHttpRequestExecutor::HttpMethod::POST
CpHttpRequestExecutor::HttpMethod::PUT
```

## Response Handling

Responses use Boost.Beast's `http::response<http::string_body>` type:

```cpp
void onResponse(const CpHttpRequestExecutor::TResponse& response)
{
  // Status code
  auto status = response.result();  // enum
  int statusCode = response.result_int();  // int (200, 404, etc.)

  // Headers
  auto contentType = response[boost::beast::http::field::content_type];

  // Body
  std::string body = response.body();

  // Check success
  if (status == boost::beast::http::status::ok)
  {
    RCLCPP_INFO(getLogger(), "Success!");
  }
}
```

## Pure Component-Based Architecture

The http_client package uses a **pure component-based architecture** with no legacy API remnants. All functionality is accessed through components:

- **Direct component access** is the only supported pattern
- Base behaviors (`CbHttpRequestBase`, `CbHttpGetRequest`, `CbHttpPostRequest`) use components internally
- For migration history, see [MIGRATION_ARCHIVED.md](MIGRATION_ARCHIVED.md)

## Examples

### Example 1: Simple GET Request

```cpp
class StFetchData : public smacc2::SmaccState<StFetchData, SmExample>
{
public:
  using SmaccState::SmaccState;

  void runtimeConfigure() override
  {
    this->configure<cl_http::CbHttpGetRequest>();
  }

  // Transitions
  using Transition = smacc2::transition<EvCbSuccess<CbHttpGetRequest>, StNextState>;
};
```

### Example 2: POST with Custom Response Handling

```cpp
class CbApiSubmit : public cl_http::CbHttpPostRequest
{
public:
  void onEntry() override
  {
    // Set up POST data
    requestExecutor_->executeRequest(
      CpHttpRequestExecutor::HttpMethod::POST,
      "/api/submit",
      R"({"username": "robot", "action": "navigate"})",
      {{"Content-Type", "application/json"}}
    );
  }

  void onResponseReceived(const CpHttpRequestExecutor::TResponse& response) override
  {
    if (response.result() == boost::beast::http::status::created)
    {
      RCLCPP_INFO(getLogger(), "Resource created successfully");
      this->postSuccessEvent();
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "API error: %d", response.result_int());
      this->postFailureEvent();
    }
  }
};
```

### Example 3: Multiple Sequential Requests

```cpp
class CbMultiRequest : public smacc2::SmaccAsyncClientBehavior
{
public:
  void onEntry() override
  {
    this->requiresComponent(requestExecutor_);

    // First request
    requestExecutor_->executeRequest(
      CpHttpRequestExecutor::HttpMethod::GET,
      "/api/status"
    );
  }

  void onResponse(const CpHttpRequestExecutor::TResponse& response)
  {
    if (requestCount_ == 0)
    {
      // Handle first response, make second request
      requestCount_++;
      requestExecutor_->executeRequest(
        CpHttpRequestExecutor::HttpMethod::POST,
        "/api/update",
        response.body()  // Use first response as second request body
      );
    }
    else
    {
      // All requests complete
      this->postSuccessEvent();
    }
  }

private:
  int requestCount_ = 0;
  CpHttpRequestExecutor* requestExecutor_;
};
```

## Dependencies

- **Boost.Beast**: HTTP/HTTPS client implementation
- **Boost.Asio**: Asynchronous I/O
- **OpenSSL**: HTTPS/TLS support
- **SMACC2**: State machine framework

## Testing

Run the reference state machine:

```bash
ros2 launch sm_atomic_http sm_atomic_http.launch
```

This will make GET requests to https://example.com and demonstrate the HTTP client functionality.

## Architecture Benefits

1. **Single Responsibility**: Each component has a focused purpose
2. **Testability**: Components can be tested independently
3. **Reusability**: Components can be composed differently for different use cases
4. **Thread Safety**: Proper strand usage prevents race conditions
5. **Lifetime Management**: State machine scoped components ensure proper cleanup
6. **SMACC2 Best Practices**: Follows recommended patterns for client libraries

## Troubleshooting

### SSL Errors

If you encounter SSL certificate errors:

```
SSL handshake failed: certificate verify failed
```

This is expected for self-signed certificates. The client currently validates certificates. For testing with self-signed certs, you may need to configure OpenSSL context (advanced usage).

### Connection Timeouts

The default timeout is 1500ms. Adjust when creating the client:

```cpp
this->createClient<cl_http::ClHttp>("https://slow-api.com", 5000);  // 5 second timeout
```

### Port Issues

Ensure the correct port is specified:
- HTTP: default port 80
- HTTPS: default port 443
- Custom: specify in URL (`https://api.example.com:8443`)

## License

Copyright 2023-2024 RobosoftAI Inc.

Licensed under the Apache License, Version 2.0.
