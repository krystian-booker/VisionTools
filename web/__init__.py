# web/__init__.py

import os
from flask import Flask
from .api    import api_bp
from .views  import views_bp

def create_app():
    app = Flask(
        __name__,
        static_folder='static',
        template_folder='templates'
    )

    app.config.update(
        TEMPLATES_AUTO_RELOAD=True,
        SEND_FILE_MAX_AGE_DEFAULT=0
    )

    app.register_blueprint(api_bp,   url_prefix='/api')
    app.register_blueprint(views_bp)

    return app
