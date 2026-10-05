"""HTTP routes for the fleet dashboard."""
import json
from http.server import BaseHTTPRequestHandler
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from pinky_fleet.fleet_common import CommandError

def handler_for(fleet):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, status, body, content_type='application/json; charset=utf-8'):
            payload = body if isinstance(body, bytes) else json.dumps(body, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(payload)

        def do_GET(self):
            if self.path == '/':
                page = Path(get_package_share_directory('pinky_fleet')) / 'web' / 'fleet.html'
                self.send(200, page.read_bytes(), 'text/html; charset=utf-8')
            elif self.path == '/api/state':
                self.send(200, fleet.state())
            elif self.path == '/api/map':
                map_id, data = fleet.map()
                self.send(200, dict(id=map_id, map=data))
            else:
                self.send(404, dict(error='Not found'))

        def do_POST(self):
            try:
                if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise ValueError('Expected application/json')
                parts = self.path.strip('/').split('/')
                if len(parts) != 4 or parts[:2] != ['api', 'robots']:
                    raise CommandError('Unknown route', 'unknown_route')
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 4096:
                    raise ValueError('Invalid request size')
                body = json.loads(self.rfile.read(length))
                message = fleet.command(parts[2], parts[3], body)
                self.send(200, dict(success=True, message=message))
            except (ValueError, KeyError, TypeError) as exc:
                code = exc.code if isinstance(exc, CommandError) else None
                self.send(400, dict(success=False, error=str(exc), code=code))
            except TimeoutError as exc:
                self.send(504, dict(success=False, error=str(exc)))
            except Exception as exc:
                self.send(500, dict(success=False, error=str(exc)))
    return Handler

