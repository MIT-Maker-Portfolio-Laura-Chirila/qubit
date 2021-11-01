from flask import Flask
from flask import jsonify
app = Flask(__name__)


@app.route('/')
def index():
    return 'Hello world'

#return the coordinates read from the .txt file in a JSON form
@app.route('/coord')
def summary():
    file = open("testfile.txt", "r+")
    string = file.read()
    print(string)
    string.split(",")
    coordX, coordY = string.split(',')
    file.close()
    return jsonify(coordX, coordY)

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')

