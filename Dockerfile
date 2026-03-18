FROM python:3.10-slim

WORKDIR /app

# Install dependencies first for caching purposes
COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy source code
COPY backend /app/backend
COPY frontend /app/frontend

# Set working directory to backend where main.py resides
WORKDIR /app/backend

# Expose Web Interface Port
EXPOSE 8000

# Start Production Server
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
