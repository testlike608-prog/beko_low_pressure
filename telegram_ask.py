"""
Ask a question in a Telegram group and return the answer.

    from telegram_ask import ask

    # Buttons question -> returns the chosen option
    action = ask("Gas leak detected! What should we do?", ["Close the valve", "Call emergency", "Ignore"])

    # Text question (no options) -> returns whatever the person types
    dummy = ask("Enter the dummy number:")

From the terminal:
    python telegram_ask.py "Enter the dummy number:"
    python telegram_ask.py "Turn on the AC?" Yes No

Note: only one program can read the bot's updates at a time, so stop bot.py
while using this file.
"""
import os
import sys
import time

import httpx
from dotenv import load_dotenv

reply = None
load_dotenv(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env"))
BOT_TOKEN = os.environ["BOT_TOKEN"]
GROUP_CHAT_ID = int(os.environ.get("GROUP_CHAT_ID", "-5387683024"))

API = f"https://api.telegram.org/bot{BOT_TOKEN}"


def _call(client: httpx.Client, method: str, **params):
    r = client.post(f"{API}/{method}", json=params, timeout=60)
    data = r.json()
    if not data.get("ok"):
        raise RuntimeError(f"Telegram {method} failed: {data.get('description')}")
    return data["result"]


def ask(question: str, options: list[str] | None = None,
        chat_id: int = GROUP_CHAT_ID, timeout: float | None = None) -> str | None:
    """Send `question` to the Telegram group and wait for the answer.

    - With `options`: shows one button per option, returns the chosen option.
    - Without `options`: waits for someone to type a message, returns that text.

    Prints the answer and returns it. Returns None if `timeout` seconds pass
    with no answer.
    """
    deadline = time.time() + timeout if timeout else None

    with httpx.Client() as client:
        # Skip old updates so we only react to answers to this question
        old = _call(client, "getUpdates", offset=-1, timeout=0)
        offset = old[-1]["update_id"] + 1 if old else 0

        if options:
            markup = {"inline_keyboard": [[{"text": o, "callback_data": str(i)}]
                                          for i, o in enumerate(options)]}
        else:
            markup = {"force_reply": True, "input_field_placeholder": "Type your answer..."}
        message_id = _call(client, "sendMessage", chat_id=chat_id, text=question,
                           reply_markup=markup)["message_id"]

        while True:
            if deadline and time.time() >= deadline:
                _call(client, "sendMessage", chat_id=chat_id, reply_to_message_id=message_id,
                      text="⌛ Time is up, no answer received")
                print("No answer (timeout)")
                return None

            wait = 30 if not deadline else max(1, min(30, int(deadline - time.time())))
            updates = _call(client, "getUpdates", offset=offset, timeout=wait,
                            allowed_updates=["callback_query" if options else "message"])

            for u in updates:
                offset = u["update_id"] + 1
                answer = None

                if options and "callback_query" in u:
                    cq = u["callback_query"]
                    if cq.get("message", {}).get("message_id") != message_id:
                        continue
                    answer = options[int(cq["data"])]
                    who = cq["from"].get("first_name", "")
                    _call(client, "answerCallbackQuery", callback_query_id=cq["id"])
                    # Remove the buttons and show who answered what
                    _call(client, "editMessageText", chat_id=chat_id, message_id=message_id,
                          text=f"{question}\n\n{who}: {answer}")

                elif not options and "message" in u:
                    m = u["message"]
                    text = m.get("text", "").strip()
                    # Any normal text sent in the group after the question (commands ignored)
                    if m["chat"]["id"] != chat_id or m["message_id"] <= message_id \
                            or not text or text.startswith("/"):
                        continue
                    answer = text
                    who = m["from"].get("first_name", "")
                    _call(client, "sendMessage", chat_id=chat_id, reply_to_message_id=m["message_id"],
                          text=f"✔️ Received from {who}: {answer}")

                if answer is not None:
                    # Confirm the offset so this update isn't delivered again
                    _call(client, "getUpdates", offset=offset, timeout=0)
                    print(answer)
                    return answer


if __name__ == "__main__":
   
    question = "Enter the dummy number:"
    reply = ask(question)
    print("Reply:", reply)
