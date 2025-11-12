# mmWavedar

# Development Guide
1. Go into the gui/ folder and run `npm install` to install dependencies.
2. Initialize the Python virtual environment.

```
python -m venv venv
. venv/Scripts/activate
pip install -r server/Requirements.txt
```
3. Start the development server with `npm run dev`.

__Note: the proxy may require you to add the following to your .env__
```
DANGEROUSLY_DISABLE_HOST_CHECK=true
```

To format the code we can use Biome in the `gui/` folder
```
npx @biomejs/biome format ./src --write
```

![test](gui/assets/test.png)

