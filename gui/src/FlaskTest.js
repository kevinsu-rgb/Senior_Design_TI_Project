import { useState, useEffect } from 'react';

export default function FlaskTest() {
  const [data, setData] = useState(null);

  useEffect(() => {
    fetch('/test')
      .then(res => res.json())
      .then(setData)
      .catch(console.error);
  }, []);

  return <p>{data?.message}</p>
}