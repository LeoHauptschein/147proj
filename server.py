from flask import Flask
from flask import request
import matplotlib.pyplot as plt

app = Flask(__name__)

tds_data = []
ph_data = []

@app.route("/")

def hello():
    tds = request.args.get("tds")
    ph = request.args.get("ph")
    oz = request.args.get("oz")
    tds_data.append(tds)
    ph_data.append(ph)
    plt.plot(tds_data)
    return f"TDS: {tds} pH: {ph} oz: {oz}"