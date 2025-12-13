# Инструкция по установке Depth-Anything-3

1. **Скачать zip-архив с ветки `master1`**  
   - Перейдите на GitHub репозиторий и скачайте ZIP-архив с ветки `master1`.

2. **Распаковать архив**  
   - Распакуйте содержимое архива в удобную папку.

3. **Скачать веса модели**  
   - Скачайте `model.safetensors` по ссылке:  
     [https://huggingface.co/depth-anything/DA3METRIC-LARGE/tree/main](https://huggingface.co/depth-anything/DA3METRIC-LARGE/tree/main)  
   - Поместите файл в папку:  
     ```
     Depth-Anything-3-main\src\depth_anything_3\weights\DA3METRIC-LARGE
     ```

4. **Скачать и установить Python 3.9.9**  
   - Ссылка для скачивания: [Python 3.9.9](https://www.python.org/downloads/release/python-399/)

5. **Создать виртуальное окружение**  
   - Откройте папку проекта в VS Code  
   - Создайте виртуальное окружение, указав путь к Python 3.9.9:
     ```bash
     "C:\...\Python39\python.exe" -m venv .venv
     ```

6. **Активировать виртуальное окружение**  
   ```bash
   .venv\Scripts\activate
   ```

7. **Установить необходимые зависимости**  
   ```bash
   pip install -r requirements.txt
   ```

8. **Установите текущую папку как Python-пакет**  
   ```bash
   pip install .
   ```

8. **Запуск файла**  
   ```bash
   python infer_camera.py
   ```
