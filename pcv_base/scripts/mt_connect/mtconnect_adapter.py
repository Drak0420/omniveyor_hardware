"""Copyright 2012, System Insights, Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License."""

"""
File taken from 
https://github.com/mtconnect/iso_digital_twin_adapter/tree/master/adapter
"""

from socketserver import ThreadingMixIn, TCPServer, BaseRequestHandler
import threading
import socket
import uuid
from datetime import datetime
import re
import struct
import sys
import traceback


class Adapter(ThreadingMixIn, TCPServer):
    allow_reuse_address = True

    def __init__(self, address, heartbeat_interval=10000, family=socket.AF_INET):
        self.address_family = family
        TCPServer.__init__(self, address, BaseRequestHandler, False)
        self._clients = dict()
        self._lock = threading.RLock()
        self._data_items = []
        self._running = False
        self._heartbeat_interval = heartbeat_interval
        self._ping_pat = re.compile("\\* PING")

    def add_data_item(self, item):
        self._data_items.append(item)

    def is_running(self):
        return self._running

    def start(self):
        self.server_bind()
        self._running = True
        self.server_activate()
        self._server_thread = threading.Thread(target=self.serve_forever)
        self._server_thread.setDaemon(True)
        print("Server started, waiting for connections on " + str(self.server_address))
        self._server_thread.start()

    def stop(self):
        self._running = False
        self.shutdown()
        for client in self._clients.values():
            client.shutdown(socket.SHUT_RDWR)
        self._server_thread.join(5.0)

    def wait_until_stopped(self):
        self._server_thread.join()

    def finish_request(self, request, client_address):
        print("Connected to " + str(client_address))
        self._lock.acquire()
        self._clients[client_address] = request
        self._lock.release()

        # Turn nageling off
        l_onoff = 1
        l_linger = 0

        request.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)
        # request.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        request.setsockopt(
            socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", l_onoff, l_linger)
        )

        self.send_initial(client_address)
        self.heartbeat(request)

        self.remove_client(client_address)

    def heartbeat(self, client):
        try:
            client.settimeout(None)
            while self._running:
                line = str(client.recv(256), "utf-8")
                if self._ping_pat.match(line):
                    if not client.gettimeout():
                        client.settimeout(self._heartbeat_interval / 500.0)
                    try:
                        self._lock.acquire()
                        client.send(
                            bytes(
                                "* PONG " + str(self._heartbeat_interval) + "\n",
                                "utf-8",
                            )
                        )
                    finally:
                        self._lock.release()
                else:
                    break
        except:
            print(
                "Exception occurred in send_to_client, removing client:"
                + str(sys.exc_info()[0])
            )
            traceback.print_tb(sys.exc_info()[2])
            print("Exception in heartbeat thread")

        print("Heartbeat thread stopped")

    def remove_client(self, client_address):
        print("Removing " + str(client_address))
        try:
            self._lock.acquire()
            if client_address in self._clients:
                socket = self._clients[client_address]
                del self._clients[client_address]
                socket.shutdown(socket.SHUT_RDWR)
        except:
            print("Exception closing socket for " + str(client_address))
        finally:
            self._lock.release()

    def begin(self):
        for di in self._data_items:
            di.begin()

    def complete(self):
        for di in self._data_items:
            di.complete()

    def sweep(self):
        for di in self._data_items:
            di.sweep()

    def unavailable(self):
        for di in self._data_items:
            di.unavailable()

    def send_initial(self, client_address):
        self.send_changed([client_address], True)

    def format_time(self):
        time = datetime.utcnow()
        return time.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"

    def send_changed(self, clients, force=False):
        text = ""
        time = self.format_time()
        separate = [item for item in self._data_items if item.separate_line()]
        combined = [item for item in self._data_items if not item.separate_line()]
        for item in combined:
            if force or item.changed():
                text += "".join(item.values(force))

        if len(text) > 0:
            self.send(time, text, clients)

        for item in separate:
            if force or item.changed():
                for line in item.values(force):
                    self.send(time, line, clients)

    def format_line(self, time, text):
        return time + text + "\n"

    def send_to_client(self, client, line):
        try:
            try:
                self._lock.acquire()
                socket = self._clients[client]
            finally:
                self._lock.release()
            if socket:
                socket.sendall(bytes(line, "utf-8"))
        except Exception as ex:
            print("Exception occurred in send_to_client, removing client:" + str(ex))
            traceback.print_tb(sys.exc_info()[2])
            self.remove_client(client)

    def send(self, time, text, clients):
        line = self.format_line(time, text)
        for client in clients:
            self.send_to_client(client, line)

    def gather(self, function):
        self.begin()

        function(self._data_items)

        self.complete()
        self.send_changed(self._clients.keys())
        self.sweep()

    def begin_gather(self):
        self.begin()

    def complete_gather(self):
        self.complete()
        self.send_changed(self._clients.keys())
        self.sweep()

    def addAsset(self, assetType, assetId, xml):
        bound = "--multiline--%s" % str(uuid.uuid4())
        text = "|@ASSET@|%s|%s|%s\n%s\n%s\n" % (assetId, assetType, bound, xml, bound)
        self.send(self.format_time(), text, self._clients.keys())

    def removeAsset(self, assetId):
        bound = "--multiline--%s" % str(uuid.uuid4())
        text = "|@REMOVE_ASSET@|%s" % assetId
        self.send(self.format_time(), text, self._clients.keys())

    def removeAllAsset(self, assetId):
        bound = "--multiline--%s" % str(uuid.uuid4())
        text = "|@REMOVE_ALL_ASSETS@|%s" % assetId
        self.send(self.format_time(), text, self._clients.keys())
