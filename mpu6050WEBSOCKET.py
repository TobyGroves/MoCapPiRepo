from flask import Flask
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app)
    
@socketio.on('my event')
def test_message(message):
    emit('my response',{'data':'gotit'})
    
@socketio.on('connect')
def test_connection():
    print('client trying to connect')
    emit('my response',{'data':'connected'})
    
@socketio.on('disconnect')
def test_disconnect():
    print('Client disconnected')


if __name__ == "__main__":
	socketio.run()
