module.exports = async function (context, req, connection) {
  context.log("Negotiate function called.");

  context.res = {
    status: 200,
    headers: {
      "Content-Type": "application/json"
    },
    body: connection
  };
};
