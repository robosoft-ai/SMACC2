^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package http_client
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.3.19 (2025-06-17)
-------------------
* OpenAI example client ported to SMACC (`#557 <https://github.com/robosoft-ai/SMACC2/issues/557>`_)
  * initial commit for openAI example
  * Replaced get with post, added key fetch logic
  * Added ability to modify the body of an http request for the http client
  * Added functionality to encode and send image data to ChatGPT
  * Added APIs to add headers to an outgoing HTTP request
  * HTTP requests can now deal with CDNs
  * Expanded example openAI node to submit a more sophisticated request with
  an image
  * Using env variable for test iamge location
  * Fixed commit errors
  * Renamed instantiation of HTTP request CB to something more specific to OpenAI
* Moved HTTP client behaviour to client library. Also added GET and POS… (`#549 <https://github.com/robosoft-ai/SMACC2/issues/549>`_)
  * Moved HTTP client behaviour to client library. Also added GET and POST-specific versions of the smacc HTTP client behaviour to make them easier to use
  * Added copyrights
  * Fixed typos
* Http client (`#522 <https://github.com/robosoft-ai/SMACC2/issues/522>`_)
  * Ported http work over from smacc1
  * Added example showasing http requests
  * Fixed some typos in the HTTP example
  * Made provision for HTTP GET/POST in the HTTP client
  * Moved HTTP client into the client_libraries folder and made separate compilation units. Modified examples to use this new client
  * Update smacc_action_client_base.hpp
  Pretty sure this change was made accidently.
  * Update smacc_action_client_base.hpp
  Messed up first commit
  * Update http_client.cpp
  Added License to file to clean up format error
  * Update http_client.cpp
  Found better Copyright & License for CI Format error.
  * Update cb_http_request.hpp
  Another little format error
  * Update sm_atomic_http.py
  Somehow a bracket became a cNode
  * - Fixed errors with HTTP requests not triggering,
  - Fixed response callback not triggering correctly and causing a crash
  - Fixed example not transitioning back to state 1 after a response has been received
  * Ported http work over from smacc1
  * Added example showasing http requests
  * Fixed some typos in the HTTP example
  * Made provision for HTTP GET/POST in the HTTP client
  * Moved HTTP client into the client_libraries folder and made separate compilation units. Modified examples to use this new client
  * Update http_client.cpp
  Added License to file to clean up format error
  * Update http_client.cpp
  Found better Copyright & License for CI Format error.
  * Update cb_http_request.hpp
  Another little format error
  * Update sm_atomic_http.py
  Somehow a bracket became a cNode
  * - Fixed errors with HTTP requests not triggering,
  - Fixed response callback not triggering correctly and causing a crash
  - Fixed example not transitioning back to state 1 after a response has been received
  * Added helper class to handle server name related interactions
  * Added SSL implementation, need to make provision for SSL and non-SSLD requests
  * Added SLL certs
  * Split the HTTp session class into own compile unit
  * Added HTTP session base class and implemented the SSL version. The client can decide between which session to initialise at runtime depending on the server set
  * Implemented non-ssl HTTP session for clients
  * Removing hard-coded ssl certs and using the default OpenSSL certs for
  verification
  * Run through formatter
  * Fix bad merge
  * Remove noisy changelog
  * Undo change that introduced and error
  * Added missing copyrights
  * Revert change to precommit config
  * Fixing copyright, formatting
  * Fixed whitespace blocking precommit
  * Revert change to precommit config
  ---------
  Co-authored-by: brettpac <brettpac@users.noreply.github.com>
* Contributors: Jaycee Lock

2.4.1 (2023-07-06)
------------------

2.3.18 (2023-07-17)
-------------------

2.3.8 (2023-03-12 21:50)
------------------------

2.3.7 (2023-03-12 21:45)
------------------------

2.3.6 (2023-03-12 20:30)
------------------------

2.3.5 (2023-03-07 00:14)
------------------------

2.3.4 (2023-03-07 00:02)
------------------------

2.3.3 (2023-03-02 22:58)
------------------------

2.3.2 (2023-03-02 22:22)
------------------------

2.3.1 (2022-11-28)
------------------

1.22.1 (2022-11-09 20:22)
-------------------------

1.22.0 (2022-11-09 19:53)
-------------------------
