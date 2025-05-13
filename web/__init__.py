# web/__init__.py

import os
from flask import Flask
from .api    import api_bp
from .views  import views_bp

def create_app():
    # 1) Create the Flask app
    app = Flask(
        __name__,
        static_folder='static',
        template_folder='templates'
    )

    # 2) Load config from .env or environment
    # e.g. app.config.from_pyfile('../.env', silent=True)
    # or use python-dotenv to auto-load .env

    # 3) Register blueprints
    app.register_blueprint(api_bp,   url_prefix='/api')
    app.register_blueprint(views_bp)  # root routes

    return app
