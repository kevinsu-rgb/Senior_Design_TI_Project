# mmWavedar

# Development Guide
1. Go into the gui/ folder and run `npm install` to install dependencies.
2. Initialize the Python virtual environment.

```
python -m venv venv
. venv/bin/activate
pip install -R server/Requirements.txt
```
3. Start the development server with `npm run dev`.

__Note: the proxy may require you to add the following to your .env__
```
DANGEROUSLY_DISABLE_HOST_CHECK=true
```

![test](gui/assets/test.png)